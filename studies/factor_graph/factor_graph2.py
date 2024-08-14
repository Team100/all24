# pylint: disable=invalid-name,too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement

import numpy as np
import gtsam  # type:ignore
from gtsam.symbol_shorthand import X, L  # type:ignore
from plot_utils import plot_result, MultivariateNormalParameters  # type:ignore


# for a real field the apriltag "landmarks" are fixed
# for this experiment say the landmarks are ([[0.5, 0.5], [0.5, 4.5]])
# and for now they are both visible all the time

prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01]))

L1 = L(1)
l1_x = np.array([2, 2])
L1prior = gtsam.PriorFactorPoint2(L1, gtsam.Point2(*l1_x), prior_noise)

L2 = L(2)
l2_x = np.array([4, 2])
L2prior = gtsam.PriorFactorPoint2(L2, gtsam.Point2(*l2_x), prior_noise)

isam = gtsam.ISAM2(gtsam.ISAM2Params())
pose_variables: list[X] = []
landmark_variables: list[L] = []
odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
measurement_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))

# x, y
prev_robot_x = np.array([0, 0])
robot_x = np.array([0, 0])


def extract_pose_marginals(variables, result) -> list:
    pose_marginals = []
    for var in variables:
        pose_marginals.append(
            MultivariateNormalParameters(
                result.atPose2(var), isam.marginalCovariance(var)
            )
        )
    return pose_marginals


def extract_landmark_marginals(variables, result) -> list:
    landmark_marginals = []
    for var in variables:
        landmark_marginals.append(
            MultivariateNormalParameters(
                result.atPoint2(var), isam.marginalCovariance(var)
            )
        )
    return landmark_marginals


def plot_variables():
    for _ in range(5):
        isam.update()
    result = isam.calculateEstimate()
    pose_marginals = extract_pose_marginals(pose_variables, result)
    landmark_marginals = extract_landmark_marginals(landmark_variables, result)
    plot_result(pose_marginals, landmark_marginals)


# start with just the landmarks
    


# robot pose iterator
x_i = 1
# initial state has only the prior to inform it
graph = gtsam.NonlinearFactorGraph()
X1 = X(x_i)
pose_variables.append(X1)
# let's say the prior is like "i have no idea"
prior_pose = gtsam.Pose2(*robot_x, 0)
# note that theta is known.
# TODO: undo that
initial_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([5.1, 5.1, 0.1]))
graph.add(gtsam.PriorFactorPose2(X1, prior_pose, initial_noise))
initial_estimate = gtsam.Values()
X1initial = gtsam.Pose2(*robot_x, 0)
initial_estimate.insert(X1, X1initial)
isam.update(graph, initial_estimate)
plot_variables()


# inform the initial state with a "bearing range factor"
graph = gtsam.NonlinearFactorGraph()
X1 = X(1)

# if we can see L1, add it to the variables and the graph
landmark_variables.append(L1)
graph.add(L1prior)

X1toL1angle = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l1_x - robot_x))))
X1toL1range = np.hypot(*(robot_x - l1_x))
graph.add(
    gtsam.BearingRangeFactor2D(
        X1,
        L1,
        X1toL1angle,
        X1toL1range,
        measurement_noise,
    )
)

# this is how new initial estimates are passed to isam
initial_estimate = gtsam.Values()
L1point = gtsam.Point2(2.00, 2.0)
initial_estimate.insert(L1, L1point)

isam.update(graph, initial_estimate)
plot_variables()


# odometry, this uses the relative pose from x1 to x2
graph = gtsam.NonlinearFactorGraph()

X1 = X(1)
X2 = X(2)
pose_variables.append(X2)

prev_robot_x = robot_x
robot_x = np.array([2, 0])
robot_delta = robot_x - prev_robot_x

X1toX2 = gtsam.Pose2(*robot_delta, 0.0)
graph.add(gtsam.BetweenFactorPose2(X1, X2, X1toX2, odometry_noise))

initial_estimate = gtsam.Values()
X2initial = gtsam.Pose2(2.30, 0.10, -0.20)
initial_estimate.insert(X2, X2initial)

isam.update(graph, initial_estimate)
plot_variables()


# now x2 is informed by the bearing-range to l1

graph = gtsam.NonlinearFactorGraph()
X2 = X(2)

graph.add(L1prior)
X2toL1angle = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l1_x - robot_x))))
X2toL1range = np.hypot(*(robot_x - l1_x))
graph.add(
    gtsam.BearingRangeFactor2D(X2, L1, X2toL1angle, X2toL1range, measurement_noise)
)
# There are no new variables this update, so no new initial values.
isam.update(graph, gtsam.Values())
plot_variables()


# now there'x x3, odomery, also informed by l2, another target.
graph = gtsam.NonlinearFactorGraph()

X2 = X(2)
X3 = X(3)

pose_variables.append(X3)
landmark_variables.append(L2)

prev_robot_x = robot_x
robot_x = np.array([4, 0])
robot_delta = robot_x - prev_robot_x

X2toX3 = gtsam.Pose2(*robot_delta, 0.0)
graph.add(gtsam.BetweenFactorPose2(X2, X3, X2toX3, odometry_noise))
graph.add(L2prior)

X3toL2angle = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l2_x - robot_x))))
X3toL2range = np.hypot(*(l2_x - robot_x))
graph.add(
    gtsam.BearingRangeFactor2D(X3, L2, X3toL2angle, X3toL2range, measurement_noise)
)

# Set initial estimates only for the new variables.
initial_estimate = gtsam.Values()
initial_estimate.insert(X3, gtsam.Pose2(4.10, 0.10, 0.10))
initial_estimate.insert(L2, gtsam.Point2(4.10, 1.80))

isam.update(graph, initial_estimate)
plot_variables()
