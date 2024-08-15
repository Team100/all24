# pylint: disable=invalid-name,too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement

import math
import time
import numpy as np
import gtsam  # type:ignore
import gtsam_unstable  # type:ignore
from gtsam.symbol_shorthand import X, L  # type:ignore
from landmark import Landmark
from plot_utils import Plot

PAUSE_TIME = 1.0
ANGLE_SCALE = 0.01
LINEAR_SCALE = 0.2


def initialize_landmarks(isam, landmarks, landmark_variables) -> None:
    # landmark position is known to within a couple of inches.
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05]))
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()
    new_timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
    for l in landmarks:
        landmark_variables.append(l.symbol)
        # this is a constant, it's rendered wrong by the plotter
        # and doesn't seem to help performance anyway
        # graph.add(gtsam.NonlinearEqualityPoint2(l.symbol, gtsam.Point2(*l.x)))
        # this allows some variation in the cameras
        # graph.add(gtsam.PriorFactorPoint2(l.symbol, gtsam.Point2(*l.x), prior_noise))
        graph.add(gtsam.PriorFactorPoint2(l.symbol, l.x, prior_noise))
        initial_estimate.insert(l.symbol, gtsam.Point2(*l.x))
        new_timestamps.insert((l.symbol, 0.0))
    isam.update(graph, initial_estimate, new_timestamps)


def initialize_robot(isam, pose_variables, robot_x, x_i) -> X:
    graph = gtsam.NonlinearFactorGraph()
    robot_X = X(x_i)
    pose_variables.append(robot_X)
    # initial_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([5.1, 5.1, 0.1]))
    initial_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
    graph.add(gtsam.PriorFactorPose2(robot_X, gtsam.Pose2(*robot_x, 0), initial_noise))
    initial_estimate = gtsam.Values()
    initial_estimate.insert(robot_X, gtsam.Pose2(*robot_x, 0))
    new_timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
    new_timestamps.insert((robot_X, 0.1))
    isam.update(graph, initial_estimate, new_timestamps)
    return robot_X


def add_odometry(isam, x_i, robot_x, robot_X, robot_delta, prev_robot_X) -> None:
    # for this demo there's more noise
    odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
    # wheel measurements are accurate to within a few millimeters
    # TODO: make noise dependent on speed
    # odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.005, 0.005, 0.005]))
    graph = gtsam.NonlinearFactorGraph()
    # TODO: rotation
    twist = gtsam.Pose2(*robot_delta, 0.0)
    # print("C ", x_i)
    graph.add(gtsam.BetweenFactorPose2(prev_robot_X, robot_X, twist, odometry_noise))
    initial_estimate = gtsam.Values()
    initial_estimate.insert(robot_X, gtsam.Pose2(*robot_x, 0.0))
    new_timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
    new_timestamps.insert((robot_X, x_i*2.0+0.1))
    # print("D ", x_i)
    try:
        isam.update(graph, initial_estimate, new_timestamps)
    except IndexError:
        print("caught IndexError in isam.update() in add_odometry()")
    except ValueError:
        print("caught ValueError in isam.update() in add_odometry()")


def add_target_sights(isam, x_i, landmarks, robot_x, robot_X) -> None:
    graph = gtsam.NonlinearFactorGraph()
    # graph.add(
    #     gtsam.PriorFactorPose2(
    #         robot_X,
    #         gtsam.Pose2(*robot_x, 0),
    #         gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1])),
    #     )
    # )
    initial_estimate = gtsam.Values()
    new_timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
    # make sure the robot is in there
    # initial_estimate.insert(robot_X, gtsam.Pose2(*robot_x, 0.0))
    # new_timestamps.insert((robot_X, x_i*2.0+0.2))
    # print("CA ", x_i)

    for l in landmarks:
        l_angle = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l.x - robot_x))))
        l_range = np.hypot(*(l.x - robot_x))
        # accuracy is proportional to distance
        # TODO: camera pixels instead of bearing range
        v = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05 * l_range, 0.3 * l_range]))
        graph.add(gtsam.BearingRangeFactor2D(robot_X, l.symbol, l_angle, l_range, v))
        # v = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3 * l_range]))
        # graph.add(gtsam.BearingFactor2D(robot_X, l.symbol, l_angle, v))
        # initial_estimate.insert(l.symbol, gtsam.Point2(*l.x))
        new_timestamps.insert((l.symbol, x_i*2.0))
    # print("CB ", x_i)
    try:
        isam.update(graph, initial_estimate, new_timestamps)
    except IndexError:
        print("caught IndexError in isam.update() in add_target_sights()")
    except ValueError:
        print("caught ValueError in isam.update() in add_target_sights()")


def forward_and_left(x_i):
    angle = ANGLE_SCALE * x_i - math.pi / 2
    return np.array([LINEAR_SCALE * math.cos(angle), LINEAR_SCALE * math.sin(angle)])


def main() -> None:

    landmarks: list[Landmark] = [
        Landmark(1, 0.5, 0.5),
        Landmark(2, 0.5, 4.5),
        # Landmark(3, 4.5, 4.5),
        # Landmark(4, 4.5, 0.5),
    ]

    # isam = gtsam.ISAM2(gtsam.ISAM2Params())

    # a small window seems to help
    lag = 40
    isam = gtsam_unstable.IncrementalFixedLagSmoother(lag)
    # isam = gtsam_unstable.BatchFixedLagSmoother(lag)
    # print(dir(isam))

    # these are for plotting
    pose_variables: list[X] = []
    landmark_variables: list[L] = []

    # x, y
    robot_x = np.array([1, 2.5])
    prev_robot_x = robot_x

    p = Plot(isam)

    initialize_landmarks(isam, landmarks, landmark_variables)
    robot_X = initialize_robot(isam, pose_variables, robot_x, 0)

    for x_i in range(10, 10000,10):
        # print("A ", x_i)
        # trim the pose variables so the plot looks better
        # pose_variables = pose_variables[-5:]
        # move the robot
        robot_delta = forward_and_left(x_i)
        robot_x = prev_robot_x + robot_delta
        prev_robot_x = robot_x

        prev_robot_X = robot_X
        robot_X = X(x_i)
        pose_variables.append(robot_X)


        t0 = time.time_ns()
        add_odometry(isam, x_i, robot_x, robot_X, robot_delta, prev_robot_X)
        add_target_sights(isam, x_i, landmarks, robot_x, robot_X)
        try:
            result = isam.calculateEstimate()
        except IndexError:
            print("caught IndexError in isam.calculateEstimate() in main()")
        pose_variables = [pv for pv in pose_variables if result.exists(pv)]

        t1 = time.time_ns()
        # print(f"timestamps: {isam.timestamps().size()}")
        if x_i % 50 == 0:
            print(f"i {x_i} duration (ns) {t1-t0}")

        p.plot_variables(result, pose_variables, landmark_variables)
        time.sleep(0.1)

        # pause occasionally
        # if x_i % 500 == 0:
        #     print("pausing so you can see it more clearly...")
        #     time.sleep(PAUSE_TIME)


if __name__ == "__main__":
    main()
