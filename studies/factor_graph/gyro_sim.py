# pylint: disable=C0103,C0114,C0115,C0116,E0611,E1101,R0904,R0913,W0621
# mypy: disable-error-code="import-untyped"

# simulate smoothing a gyro with noise and bias, with odometry.

# A single axis gyro bias is just a scalar.
# The units for stddev are σ = rad/s or rad √Hz/s
# so use Values.atDouble()

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

from numerical_derivative import numericalDerivative11

from custom_factor_type import CustomFactorType


def gyro_h_fn() -> Callable[[Pose2], float]:
    def h(robot_pose: Pose2) -> float:
        return robot_pose.rotation().theta()

    return h


# so this should be a sort of "between" factor for
# two pose values that also takes the bias value.
def GyroFactor(
    measured: float, model: SharedNoiseModel, poseKey: int
) -> NonlinearFactor:
    """Unary for now: TODO: model a real gyro"""

    def error_func(this: CustomFactor, v: Values, H: list[np.ndarray]) -> np.ndarray:
        pose2: Pose2 = v.atPose2(this.keys()[0])
        h = gyro_h_fn()
        result = h(pose2) - measured
        if H is not None:
            H[0] = numericalDerivative11(h, pose2)
        return np.array([result])

    return CustomFactor(
        CustomFactorType.UNARY.value, model, KeyVector([poseKey]), error_func
    )


def make_smoother() -> FixedLagSmoother:
    optimization_params = ISAM2GaussNewtonParams()
    optimization_params.setWildfireThreshold(0.001)
    isam_params = gtsam.ISAM2Params()
    # the crash is cause by the optimization params.
    isam_params.setOptimizationParams(optimization_params)
    # relinearizing less makes the path more consistent as new factors are added
    # relinearizing more makes it more jittery
    isam_params.relinearizeSkip = 1
    isam_params.evaluateNonlinearError = False
    isam_params.cacheLinearizedFactors = True
    return gtsam_unstable.IncrementalFixedLagSmoother(5, isam_params)


def initialize(isam) -> None:
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
    graph.add(
        gtsam.PriorFactorDouble(
            B(0), bias, Diagonal.Sigmas(np.array([0.1]))
        )
    )
    values.insert(B(0), bias)
    timestamps.insert((B(0), 0))
    isam.update(graph, values, timestamps)


def add_factors(
    isam,
    x_i: int,
    robot_x: Pose2,
) -> None:
    graph = NonlinearFactorGraph()
    values = Values()
    timestamps = gtsam.FixedLagSmootherKeyTimestampMap()

    robot_delta = gtsam.Pose2(1, 0, 0)
    new_estimate = robot_x.compose(robot_delta)
    values.insert(X(x_i), new_estimate)
    timestamps.insert((X(x_i), x_i))

    graph.add(
        BetweenFactorPose2(
            X(x_i - 1),
            X(x_i),
            robot_delta,
            Diagonal.Sigmas(np.array([0.1, 0.1, 0.1])),
        )
    )

    values.insert(B(x_i), 0)
    timestamps.insert((B(x_i), x_i))

    graph.add(
        BetweenFactorDouble(
            B(x_i - 1), B(x_i), 0, gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1]))
        )
    )

    isam.update(graph, values, timestamps)


def main() -> None:
    isam = make_smoother()
    initialize(isam)

    robot_x = gtsam.Pose2(0, 0, 0)
    for x_i in range(1, 10):
        add_factors(isam, x_i, robot_x)

        result = isam.calculateEstimate()
        print(result)


if __name__ == "__main__":
    main()
