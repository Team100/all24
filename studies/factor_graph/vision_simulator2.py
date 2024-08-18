"""
Drive around and show projected vision data
"""

# pylint: disable=too-many-locals,invalid-name,too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement

import math

import cv2
import gtsam
import gtsam_unstable  # type:ignore
from gtsam.symbol_shorthand import L, X
import matplotlib
matplotlib.use('Qt5Cairo', force=True)
import matplotlib.pyplot as plt  # type:ignore
import numpy as np
import numpy.typing as npt
from landmark import Landmark
from plot_utils import Plot

NOISE2 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))
NOISE3 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))


class Viewer:
    def __init__(self, name, window_position, width, height) -> None:
        self.fig = plt.figure(name, figsize=(6, 6))
        self.fig.canvas.manager.window.move(700 * window_position + 50, 50)
        self.ax = plt.axes(xlim=(0, width), ylim=(0, height))
        self.ax.invert_yaxis()
        self.ax.set_aspect("equal", adjustable="box")
        self.scatter = self.ax.scatter(
            [], [], marker="+", s=50, color="orangered", linewidths=1, zorder=20
        )
        self.fig.canvas.draw()
        plt.show(block=False)
        self.fig.canvas.update()
        self.fig.canvas.flush_events()

    def draw(self, draw_points) -> npt.NDArray:
        self.ax.draw_artist(self.ax.patch)
        self.scatter.set_offsets(draw_points)
        self.ax.draw_artist(self.scatter)
        self.fig.canvas.update()
        self.fig.canvas.flush_events()
        return draw_points


class Projector:
    # x offset is the usual "robot-relative" (positive left)
    def __init__(self, width, height, x_offset) -> None:
        # same as pi v2 camera
        fx = 660
        fy = 660
        cx = width // 2
        cy = height // 2
        self.camera_matrix = np.array(
            [
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1],
            ],
            np.float32,
        )
        # no distortion
        # self.dist_coeffs = np.array([0, 0, 0, 0, 0], np.float32)
        # lots of distortion (more than the pi v2 camera)
        self.dist_coeffs = np.array([0.2, 0.1, 0, 0, 0], np.float32)

        # Define the 3D point in the world coordinate system
        # note this is Z-forward, offset to camera height
        # tag center
        # offset is positive-left; camera is positive-right
        # but also world-in-camera so leave the sign alone
        x = x_offset  # centered
        y = -2  # up
        z = 4  # medium-distance
        tag_size = 0.1651  # real tag size

        self.points_3d = np.array(
            [
                [x - tag_size / 2, y - tag_size / 2, z],
                [x - tag_size / 2, y + tag_size / 2, z],
                [x + tag_size / 2, y + tag_size / 2, z],
                [x + tag_size / 2, y - tag_size / 2, z],
            ],
            np.float32,
        )
        # camera offsets
        # camera tilt up, so world tilt down
        self.rvec = np.array([[-0.43], [0], [0]], np.float32)
        # identity
        self.rmat = cv2.Rodrigues(self.rvec)[0]
        self.tvec = np.array([[0], [0], [0]], np.float32)

    # pose is (3,1)
    def project(self, pose) -> npt.NDArray:
        # camera is Z-forward
        # also tvec is "world in camera frame" so inverted
        net_tvec = self.tvec + np.array([[pose[1]], [0], [-pose[0]]], dtype=np.float32)
        # the polarity here is probably wrong
        # pose[2] is positive-left, rvec Y is positive-right, but also
        # rvec is "world in camera frame" so pose-left is world-right
        robot_rvec = np.array([0, pose[2], 0], dtype=np.float32)
        robot_rmat = cv2.Rodrigues(robot_rvec)[0]
        net_rmat = np.dot(robot_rmat, self.rmat)
        net_rvec = cv2.Rodrigues(net_rmat)[0]

        points_2d, _ = cv2.projectPoints(
            self.points_3d,
            net_rvec,
            net_tvec,
            self.camera_matrix,
            self.dist_coeffs,
        )

        return np.squeeze(points_2d)


def initialize(isam, x, y) -> None:
    graph = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
    
    graph.add(gtsam.PriorFactorPoint2(L(0), [4,0], NOISE2))
    values.insert(L(0), gtsam.Point2(4,0))
    timestamps.insert((L(0), 0))
    
    graph.add(gtsam.PriorFactorPose2(X(0), gtsam.Pose2(x, y, 0), NOISE3))
    values.insert(X(0), gtsam.Pose2(x, y, 0))
    timestamps.insert((X(0), 0))
    isam.update(graph, values, timestamps)


def main() -> None:
    N = 500
    width = 832
    height = 616

    # these are facing parallel
    p0 = Projector(width, height, 0.5)
    v0 = Viewer("left eye", 0, width, height)
    p1 = Projector(width, height, -0.5)
    v1 = Viewer("right eye", 1, width, height)
    isam = gtsam_unstable.IncrementalFixedLagSmoother(10)
    x0 = Plot(isam, "x0", 2)
    initialize(isam, 0, 0)
    pose_variables: list[X] = [X(0)]
    landmarks: list[Landmark] = [Landmark(0, 4, 0)]



    print("dolly (X), initially forward")
    for i in np.linspace(0, 2 * math.pi, N):
        pose = np.array([math.sin(i) * 2, 0, 0], dtype=np.float32)
        j0 = p0.project(pose)
        v0.draw(j0)
        j1 = p1.project(pose)
        v1.draw(j1)
        result = isam.calculateEstimate()
        x0.plot_variables(result, pose_variables, landmarks)


    print("truck (Y), initially left")
    for i in np.linspace(0, 2 * math.pi, N):
        pose = np.array([0, math.sin(i) * 2, 0], dtype=np.float32)
        j0 = p0.project(pose)
        v0.draw(j0)
        j1 = p1.project(pose)
        v1.draw(j1)
        result = isam.calculateEstimate()
        x0.plot_variables(result, pose_variables, landmarks)

    print("pan (theta), initially left")
    for i in np.linspace(0, 2 * math.pi, N):
        pose = np.array([0, 0, math.sin(i) * math.pi / 6], dtype=np.float32)
        j0 = p0.project(pose)
        v0.draw(j0)
        j1 = p1.project(pose)
        v1.draw(j1)
        result = isam.calculateEstimate()
        x0.plot_variables(result, pose_variables, landmarks)

    print("drive in an XY circle")
    for i in np.linspace(0, 2 * math.pi, N):
        pose = np.array([math.cos(i) * 2, math.sin(i) * 2, 0], dtype=np.float32)
        j0 = p0.project(pose)
        v0.draw(j0)
        j1 = p1.project(pose)
        v1.draw(j1)
        result = isam.calculateEstimate()
        x0.plot_variables(result, pose_variables, landmarks)


if __name__ == "__main__":
    main()
