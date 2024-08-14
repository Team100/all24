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
l1_x = np.array([0.5, 0.5])
L1prior = gtsam.PriorFactorPoint2(L1, gtsam.Point2(*l1_x), prior_noise)

L2 = L(2)
l2_x = np.array([0.5, 4.5])
L2prior = gtsam.PriorFactorPoint2(L2, gtsam.Point2(*l2_x), prior_noise)

isam = gtsam.ISAM2(gtsam.ISAM2Params())
pose_variables: list[X] = []
landmark_variables: list[L] = []
odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
# measurement noise
v = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))

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
    # it takes a lot of iterations to get over bad initial estimates
    for _ in range(5):
        isam.update()
    result = isam.calculateEstimate()
    pose_marginals = extract_pose_marginals(pose_variables, result)
    landmark_marginals = extract_landmark_marginals(landmark_variables, result)
    plot_result(pose_marginals, landmark_marginals)


###############################

# start with just the landmarks
landmark_variables.append(L1)
landmark_variables.append(L2)
graph = gtsam.NonlinearFactorGraph()
graph.add(L1prior)
graph.add(L2prior)
initial_estimate = gtsam.Values()
L1point = gtsam.Point2(*l1_x)
L2point = gtsam.Point2(*l2_x)
initial_estimate.insert(L1, L1point)
initial_estimate.insert(L2, L2point)
isam.update(graph, initial_estimate)
plot_variables()

##################################

x_i = 1
graph = gtsam.NonlinearFactorGraph()
X1 = X(x_i)
pose_variables.append(X1)

initial_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([5.1, 5.1, 0.1]))
graph.add(gtsam.PriorFactorPose2(X1, gtsam.Pose2(*robot_x, 0), initial_noise))

X1toL1a = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l1_x - robot_x))))
X1toL1r = np.hypot(*(l1_x - robot_x))
graph.add(gtsam.BearingRangeFactor2D(X1, L1, X1toL1a, X1toL1r, v))
X1toL2a = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l2_x - robot_x))))
X1toL2r = np.hypot(*(l2_x - robot_x))
graph.add(gtsam.BearingRangeFactor2D(X1, L2, X1toL2a, X1toL2r, v))

initial_estimate = gtsam.Values()
initial_estimate.insert(X1, gtsam.Pose2(*robot_x, 0))

isam.update(graph, initial_estimate)
plot_variables()

#################################

x_i += 1
graph = gtsam.NonlinearFactorGraph()
X2 = X(x_i)
pose_variables.append(X2)

prev_robot_x = robot_x
robot_x = np.array([2, 0])
robot_delta = robot_x - prev_robot_x

X1toX2 = gtsam.Pose2(*robot_delta, 0.0)
graph.add(gtsam.BetweenFactorPose2(X1, X2, X1toX2, odometry_noise))

X2toL1 = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l1_x - robot_x))))
X2toL1r = np.hypot(*(l1_x - robot_x))
graph.add(gtsam.BearingRangeFactor2D(X2, L1, X2toL1, X2toL1r, v))
X2toL2a = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l2_x - robot_x))))
X2toL2r = np.hypot(*(l2_x - robot_x))
graph.add(gtsam.BearingRangeFactor2D(X2, L2, X2toL2a, X2toL2r, v))


initial_estimate = gtsam.Values()
initial_estimate.insert(X2, gtsam.Pose2(*robot_x, 0))

isam.update(graph, initial_estimate)
plot_variables()

#########################################

x_i += 1
graph = gtsam.NonlinearFactorGraph()
X3 = X(x_i)
pose_variables.append(X3)

prev_robot_x = robot_x
robot_x = np.array([4, 0])
robot_delta = robot_x - prev_robot_x

X2toX3 = gtsam.Pose2(*robot_delta, 0.0)
graph.add(gtsam.BetweenFactorPose2(X2, X3, X2toX3, odometry_noise))

X3toL1angle = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l1_x - robot_x))))
X3toL1range = np.hypot(*(l1_x - robot_x))
graph.add(gtsam.BearingRangeFactor2D(X3, L1, X3toL1angle, X3toL1range, v))

X3toL2angle = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l2_x - robot_x))))
X3toL2range = np.hypot(*(l2_x - robot_x))
graph.add(gtsam.BearingRangeFactor2D(X3, L2, X3toL2angle, X3toL2range, v))


initial_estimate = gtsam.Values()
initial_estimate.insert(X3, gtsam.Pose2(*robot_x, 0.0))

isam.update(graph, initial_estimate)
plot_variables()
