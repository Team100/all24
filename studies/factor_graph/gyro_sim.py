# pylint: disable=C0103,C0114,C0115,C0116,E0611,E1101,R0904,R0913,W0621
# mypy: disable-error-code="import-untyped"

# simulate smoothing a gyro with noise and bias, with odometry.

# A single axis gyro bias is just a scalar.
# The units for stddev are σ = rad/s or rad √Hz/s
# so use Values.atDouble()

import math

from typing import Callable

import gtsam.noiseModel
import numpy as np
import gtsam
import gtsam_unstable

from gtsam import Point2, Point3, Pose3, Rot3, Pose2, Rot2
from gtsam import CustomFactor, NonlinearFactor, KeyVector
from gtsam import NonlinearFactorGraph, Values
from gtsam import FixedLagSmoother, ISAM2GaussNewtonParams
from gtsam import BetweenFactorPose2, BetweenFactorDouble

from gtsam.noiseModel import Diagonal
from gtsam.noiseModel import Base as SharedNoiseModel

from gtsam.symbol_shorthand import B  # gyro bias
from gtsam.symbol_shorthand import X  # robot pose

from numerical_derivative import numericalDerivative31
from numerical_derivative import numericalDerivative32
from numerical_derivative import numericalDerivative33

from custom_factor_type import CustomFactorType


# the gyro measurement function, "h" yields omega
# arg here is singleton array because of how autodiff works
def gyro_h_fn(dt_s) -> Callable[[np.ndarray, Pose2, Pose2], float]:
    def h(bias: np.ndarray, prev_pose: Pose2, curr_pose: Pose2) -> float:
        prev_theta = prev_pose.rotation().theta()
        curr_theta = curr_pose.rotation().theta()
        return bias[0] + (curr_theta - prev_theta) / dt_s

    return h


# so this should be a sort of "between" factor for
# two pose values that also takes the bias value.
def GyroFactor(
    dt_s: float,  # time step, sec
    measured_omega_rad_s: float,  # actual gyro measurement, rad/s
    model: SharedNoiseModel,  # h noise; h is scalar.
    bias_key: int,  # current bias value, rad/s
    prev_pose_key: int,
    curr_pose_key: int,
) -> NonlinearFactor:
    def error_func(this: CustomFactor, v: Values, H: list[np.ndarray]) -> np.ndarray:
        bias: np.ndarray = np.array([v.atDouble(this.keys()[0])])
        prev_pose: Pose2 = v.atPose2(this.keys()[1])
        curr_pose: Pose2 = v.atPose2(this.keys()[2])
        h = gyro_h_fn(dt_s)
        result = h(bias, prev_pose, curr_pose) - measured_omega_rad_s
        if H is not None:
            H[0] = numericalDerivative31(h, bias, prev_pose, curr_pose)
            H[1] = numericalDerivative32(h, bias, prev_pose, curr_pose)
            H[2] = numericalDerivative33(h, bias, prev_pose, curr_pose)
        return np.array([result])

    return CustomFactor(
        CustomFactorType.OTHER.value,
        model,
        KeyVector([bias_key, prev_pose_key, curr_pose_key]),
        error_func,
    )


def make_smoother() -> FixedLagSmoother:
    optimization_params = ISAM2GaussNewtonParams()
    optimization_params.setWildfireThreshold(0.001)
    isam_params = gtsam.ISAM2Params()
    isam_params.setOptimizationParams(optimization_params)
    # relinearizing less makes the path more consistent as new factors are added
    # relinearizing more makes it more jittery
    isam_params.relinearizeSkip = 1
    isam_params.evaluateNonlinearError = False
    isam_params.cacheLinearizedFactors = True
    return gtsam_unstable.IncrementalFixedLagSmoother(20, isam_params)


def initialize(isam) -> None:
    """Add priors for robot pose, X, and gyro bias, B."""
    graph = NonlinearFactorGraph()
    values = Values()
    timestamps = gtsam.FixedLagSmootherKeyTimestampMap()
    robot_x = gtsam.Pose2(0, 0, 0)
    graph.add(
        gtsam.PriorFactorPose2(
            X(0),
            robot_x,
            Diagonal.Sigmas(np.array([0.1, 0.1, 0.1])),
        )
    )
    values.insert(X(0), robot_x)
    timestamps.insert((X(0), 0))

    bias = 0
    graph.add(gtsam.PriorFactorDouble(B(0), bias, Diagonal.Sigmas(np.array([0.1]))))
    values.insert(B(0), bias)
    timestamps.insert((B(0), 0))
    isam.update(graph, values, timestamps)


SAMPLE_RATE_HZ = 50.0
TICK_S = 1 / SAMPLE_RATE_HZ
SPEED_M_S = 0.1
# these are in the canonical units
GYRO_BIAS = 0.05  # bias in simulated omega measurement, rad*sqrt(hz)/s
GYRO_NOISE = 0.05  # noise in simulated omega measurement, rad/sqrt(hz)s
RNG = np.random.RandomState(0)


def main() -> None:
    isam = make_smoother()
    initialize(isam)

    robot_gt = gtsam.Pose2(0, 0, 0)  # simulated robot pose
    gyro_bias_rad_s = 0.0  # random walk noise
    for x_i in range(1, 50):

        graph = NonlinearFactorGraph()
        values = Values()
        timestamps = gtsam.FixedLagSmootherKeyTimestampMap()

        # move in X, don't rotate
        robot_delta = gtsam.Pose2(SPEED_M_S * TICK_S, 0, 0)
        robot_gt = robot_gt.compose(robot_delta)
        values.insert(X(x_i), robot_gt)
        timestamps.insert((X(x_i), x_i))
        graph.add(
            BetweenFactorPose2(
                X(x_i - 1),
                X(x_i),
                robot_delta,  # measurement is just the ground truth
                Diagonal.Sigmas(np.array([0.1, 0.1, 0.1])),
            )
        )

        # the simulated gyro, not known to the model

        # bias is rad*sqrt(hz)/s so divide by hz to get rad/s
        gyro_bias_sigma = GYRO_BIAS / math.sqrt(SAMPLE_RATE_HZ)
        # gyro_bias_rad_s += gyro_bias_sigma * RNG.standard_normal()

        # for now make gyro bias a (large)constant, to see if the model will learn it.
        gyro_bias_rad_s = 0.01
        # noise is rad/sqrt(hz)*s so multiply by hz to get rad/s
        gyro_white_sigma = GYRO_NOISE * math.sqrt(SAMPLE_RATE_HZ)
        gyro_white_noise_rad_s = gyro_white_sigma * RNG.standard_normal()
        # for now no noise
        gyro_white_noise_rad_s = 0.0

        # robot delta rotation rate is zero
        measured_omega_rad_s = (
            robot_delta.rotation().theta() / TICK_S
            + gyro_bias_rad_s
            + gyro_white_noise_rad_s
        )

        # this is the model's estimate of the bias (rad/s)
        # every time step has a different bias number
        # close to the previous
        values.insert(B(x_i), 0)
        timestamps.insert((B(x_i), x_i))
        graph.add(
            BetweenFactorDouble(
                B(x_i - 1),
                B(x_i),
                0,  # expected = close to previous
                # Diagonal.Sigmas(np.array([gyro_bias_sigma])),
                Diagonal.Sigmas(np.array([1])),
            )
        )

        # the actual gyro measurement

        graph.add(
            GyroFactor(
                TICK_S,
                measured_omega_rad_s,
                # Diagonal.Sigmas(np.array([gyro_white_sigma]))
                Diagonal.Sigmas(np.array([0.001])),
                B(x_i),
                X(x_i - 1),
                X(x_i),
            )
        )

        isam.update(graph, values, timestamps)

        result = isam.calculateEstimate()
        print(result)
        print("measured omega rad/s ", measured_omega_rad_s)
        print("gt pose ", robot_gt)


if __name__ == "__main__":
    main()
