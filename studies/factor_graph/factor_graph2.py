# pylint: disable=invalid-name,too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement

import time
import numpy as np
import gtsam  # type:ignore
from gtsam.symbol_shorthand import X, L  # type:ignore
from plot_utils import Plot

FRAME_TIME = 0.1

# for a real field the apriltag "landmarks" are fixed
# for this experiment say the landmarks are ([[0.5, 0.5], [0.5, 4.5]])
# and for now they are both visible all the time

prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05]))

L1 = L(1)
l1_x = np.array([0.5, 0.5])


L2 = L(2)
l2_x = np.array([0.5, 4.5])


isam = gtsam.ISAM2(gtsam.ISAM2Params())
pose_variables: list[X] = []
landmark_variables: list[L] = []
odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
# measurement noise
v = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))

# x, y
prev_robot_x = np.array([0, 0])
robot_x = np.array([0, 0])

# initialize the plotter

p = Plot(isam)

###############################

# start with just the landmarks
landmark_variables.append(L1)
landmark_variables.append(L2)
graph = gtsam.NonlinearFactorGraph()
graph.add(gtsam.PriorFactorPoint2(L1, gtsam.Point2(*l1_x), prior_noise))
graph.add(gtsam.PriorFactorPoint2(L2, gtsam.Point2(*l2_x), prior_noise))
initial_estimate = gtsam.Values()
initial_estimate.insert(L1, gtsam.Point2(*l1_x))
initial_estimate.insert(L2, gtsam.Point2(*l2_x))
isam.update(graph, initial_estimate)
p.plot_variables(pose_variables, landmark_variables)
time.sleep(FRAME_TIME)

################################

# initialize the robot position
x_i = 0
graph = gtsam.NonlinearFactorGraph()
robot_X = X(x_i)
pose_variables.append(robot_X)
initial_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([5.1, 5.1, 0.1]))
graph.add(gtsam.PriorFactorPose2(robot_X, gtsam.Pose2(*robot_x, 0), initial_noise))
initial_estimate = gtsam.Values()
initial_estimate.insert(robot_X, gtsam.Pose2(*robot_x, 0))
isam.update(graph, initial_estimate)
# it takes a lot of iterations to get over bad initial estimates
# it's ok to update more to fix that.
# for _ in range(5):
#     isam.update()
p.plot_variables(pose_variables, landmark_variables)
time.sleep(FRAME_TIME)

##################################

# add a target sight, without changing the robot position
graph = gtsam.NonlinearFactorGraph()
X1toL1a = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l1_x - robot_x))))
X1toL1r = np.hypot(*(l1_x - robot_x))
graph.add(gtsam.BearingRangeFactor2D(robot_X, L1, X1toL1a, X1toL1r, v))
isam.update(graph, gtsam.Values())
p.plot_variables(pose_variables, landmark_variables)
time.sleep(FRAME_TIME)

##################################

# add another target sight, without changing the robot position
graph = gtsam.NonlinearFactorGraph()
X1toL2a = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l2_x - robot_x))))
X1toL2r = np.hypot(*(l2_x - robot_x))
graph.add(gtsam.BearingRangeFactor2D(robot_X, L2, X1toL2a, X1toL2r, v))
isam.update(graph, gtsam.Values())
p.plot_variables(pose_variables, landmark_variables)
time.sleep(FRAME_TIME)

#################################

# move the robot
prev_robot_x = robot_x
robot_x = np.array([2, 0])
robot_delta = robot_x - prev_robot_x
x_i += 1
prev_robot_X = robot_X
robot_X = X(x_i)
pose_variables.append(robot_X)
graph = gtsam.NonlinearFactorGraph()
X1toX2 = gtsam.Pose2(*robot_delta, 0.0)
graph.add(gtsam.BetweenFactorPose2(prev_robot_X, robot_X, X1toX2, odometry_noise))
initial_estimate = gtsam.Values()
initial_estimate.insert(robot_X, gtsam.Pose2(*robot_x, 0))
isam.update(graph, initial_estimate)
p.plot_variables(pose_variables, landmark_variables)
time.sleep(FRAME_TIME)

#########################################

# add target sight
graph = gtsam.NonlinearFactorGraph()
X2toL1 = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l1_x - robot_x))))
X2toL1r = np.hypot(*(l1_x - robot_x))
graph.add(gtsam.BearingRangeFactor2D(robot_X, L1, X2toL1, X2toL1r, v))
isam.update(graph, gtsam.Values())
p.plot_variables(pose_variables, landmark_variables)
time.sleep(FRAME_TIME)

#########################################

# add target sight
graph = gtsam.NonlinearFactorGraph()
X2toL2a = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l2_x - robot_x))))
X2toL2r = np.hypot(*(l2_x - robot_x))
graph.add(gtsam.BearingRangeFactor2D(robot_X, L2, X2toL2a, X2toL2r, v))
isam.update(graph, gtsam.Values())
p.plot_variables(pose_variables, landmark_variables)
time.sleep(FRAME_TIME)

################################

# move the robot
prev_robot_x = robot_x
robot_x = np.array([4, 0])
robot_delta = robot_x - prev_robot_x
x_i += 1
prev_robot_X = robot_X
robot_X = X(x_i)
pose_variables.append(robot_X)
graph = gtsam.NonlinearFactorGraph()
X2toX3 = gtsam.Pose2(*robot_delta, 0.0)
graph.add(gtsam.BetweenFactorPose2(prev_robot_X, robot_X, X2toX3, odometry_noise))
initial_estimate = gtsam.Values()
initial_estimate.insert(robot_X, gtsam.Pose2(*robot_x, 0.0))
isam.update(graph, initial_estimate)
p.plot_variables(pose_variables, landmark_variables)
time.sleep(FRAME_TIME)

##############################

# add target sight
graph = gtsam.NonlinearFactorGraph()
X3toL1angle = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l1_x - robot_x))))
X3toL1range = np.hypot(*(l1_x - robot_x))
graph.add(gtsam.BearingRangeFactor2D(robot_X, L1, X3toL1angle, X3toL1range, v))
isam.update(graph, gtsam.Values())
p.plot_variables(pose_variables, landmark_variables)
time.sleep(FRAME_TIME)

##############################

# add target sight
graph = gtsam.NonlinearFactorGraph()
X3toL2angle = gtsam.Rot2.fromAngle(np.arctan2(*np.flip((l2_x - robot_x))))
X3toL2range = np.hypot(*(l2_x - robot_x))
graph.add(gtsam.BearingRangeFactor2D(robot_X, L2, X3toL2angle, X3toL2range, v))
isam.update(graph, gtsam.Values())
p.plot_variables(pose_variables, landmark_variables)
p.wait()

##############################

# todo: add timing
t0 = time.time_ns()
t1 = time.time_ns()
duration = t1 - t0
print(f"duration ns {duration}")
