# pylint: disable=invalid-name,too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement

import math
import time
import numpy as np
import gtsam  # type:ignore
import gtsam_unstable  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore
from landmark import Landmark
from plot_utils import Plot

PAUSE_TIME = 1.0
ANGLE_SCALE = 0.1
LINEAR_SCALE = 0.2

# noise is really high to make the animation more interesting
NOISE2 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2]))
NOISE3 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.5, 0.5, 0.5]))


def initialize(isam, landmarks, robot_x) -> None:
    graph = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()

    for l in landmarks:
        graph.add(gtsam.PriorFactorPoint2(l.symbol, l.x, NOISE2))
        values.insert(l.symbol, gtsam.Point2(*l.x))
        timestamps.insert((l.symbol, 0))

    graph.add(gtsam.PriorFactorPose2(X(0), gtsam.Pose2(*robot_x, 0), NOISE3))
    values.insert(X(0), gtsam.Pose2(*robot_x, 0))
    timestamps.insert((X(0), 0))
    isam.update(graph, values, timestamps)


def add_odometry_and_target_sights(isam, x_i, robot_x, robot_delta, landmarks) -> None:
    graph = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
    twist = gtsam.Pose2(*robot_delta, 0.0)
    graph.add(gtsam.BetweenFactorPose2(X(x_i - 1), X(x_i), twist, NOISE3))
    values.insert(X(x_i), gtsam.Pose2(*robot_x, 0.0))
    timestamps.insert((X(x_i), x_i))
    for l in landmarks:
        l_angle = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l.x - robot_x))))
        l_range = np.hypot(*(l.x - robot_x))
        graph.add(
            gtsam.BearingRangeFactor2D(X(x_i), l.symbol, l_angle, l_range, NOISE2)
        )
        timestamps.insert((l.symbol, x_i))
    isam.update(graph, values, timestamps)


def forward_and_left(x_i):
    angle = ANGLE_SCALE * x_i - math.pi / 2
    return np.array([LINEAR_SCALE * math.cos(angle), LINEAR_SCALE * math.sin(angle)])


def main() -> None:
    landmarks: list[Landmark] = [Landmark(0, 0.5, 0.5), Landmark(1, 0.5, 4.5)]
    isam = gtsam_unstable.IncrementalFixedLagSmoother(6)
    p = Plot(isam)
    robot_x = np.array([1, 2.5])
    prev_robot_x = robot_x
    initialize(isam, landmarks, robot_x)
    pose_variables: list[X] = [X(0)]
    for x_i in range(1, 100):
        robot_delta = forward_and_left(x_i)
        robot_x = prev_robot_x + robot_delta
        prev_robot_x = robot_x
        t0 = time.time_ns()
        # only one isam.update() is allowed per time step
        add_odometry_and_target_sights(isam, x_i, robot_x, robot_delta, landmarks)
        result = isam.calculateEstimate()
        pose_variables.append(X(x_i))
        pose_variables = [pv for pv in pose_variables if result.exists(pv)]
        t1 = time.time_ns()
        if x_i % 5 == 0:
            print(f"i {x_i} duration (ns) {t1-t0}")
        p.plot_variables(result, pose_variables, landmarks)


if __name__ == "__main__":
    main()
