# pylint: disable=C0301,E0611,R0902,R0903
# mypy: disable-error-code="import-untyped"
"""Minimal simulated telemetry generator for both smoothing and calibration.

The field is 4x4 meters
The field has one tag on the far (+x) end, 1m off the floor
The robot has one camera with the same pose as the robot, except 1m high.
The robot path is a circle of radius 1m, centered 3m away from the tag, followed at 1m/s.
The robot rotates back and forth with an amplitude of 0.5 rad, at 3x the frequency of the circular path (so 4 rad/s)

Because the gyro drift is a random walk, the whole thing needs to be sequential.

Woah i have no idea how to compute the acceleration for this path, so it's not there.

TODO:
use realistic 6mm lens and GS sensor parameters.
add camera noise, white noise +/- 1 px
add gyro noise, random walk, plus white noise +/- 0.01 rad.
"""

import math

import numpy as np
from gtsam import Cal3DS2  # includes distortion
from gtsam import (PinholeCameraCal3DS2, Point2, Point3, Pose2,  # type:ignore
                   Pose3, Rot3)
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from app.pose_estimator.swerve_drive_kinematics import SwerveDriveKinematics100
from app.pose_estimator.swerve_module_position import (OptionalRotation2d,
                                                       SwerveModulePosition100)

TAG_SIZE_M: float = 0.1651
TAG_X: float = 4
TAG_Y: float = 0
TAG_Z: float = 1
PATH_CENTER_X_M = 1
PATH_CENTER_Y_M = 0
PATH_RADIUS_M = 1
PATH_PERIOD_S = 2.0 * math.pi
PAN_PERIOD_S = PATH_PERIOD_S / 3
PAN_SCALE_RAD = 0.5
# camera "zero" is facing +z; this turns it to face +x
CAM_COORD = Pose3(
    Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
    Point3(0, 0, 0),  # type: ignore
)


class CircleSimulator:
    def __init__(self) -> None:
        # TODO: actual wheelbase etc
        self.kinematics = SwerveDriveKinematics100(
            [
                Translation2d(0.5, 0.5),
                Translation2d(0.5, -0.5),
                Translation2d(-0.5, 0.5),
                Translation2d(-0.5, -0.5),
            ]
        )
        self.positions = [
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        ]
        # cheating the initial pose
        # TODO: more clever init
        self.wpi_pose = Pose2d(PATH_CENTER_X_M + PATH_RADIUS_M, 0, 0)
        self.time_s: float = 0
        self.gt_x: float
        self.gt_y: float
        self.gt_theta: float
        # the order is the same as the detector getCorners order
        # lower left
        # lower right
        # upper right
        # upper left
        self.gt_pixels: list[Point2]

        # set positions back to zero
        # TODO: more clever init
        self.positions = [
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        ]
        # constant landmark points
        self.l0 = np.array([TAG_X, TAG_Y + (TAG_SIZE_M / 2), TAG_Z - (TAG_SIZE_M / 2)])
        self.l1 = np.array([TAG_X, TAG_Y - (TAG_SIZE_M / 2), TAG_Z - (TAG_SIZE_M / 2)])
        self.l2 = np.array([TAG_X, TAG_Y - (TAG_SIZE_M / 2), TAG_Z + (TAG_SIZE_M / 2)])
        self.l3 = np.array([TAG_X, TAG_Y + (TAG_SIZE_M / 2), TAG_Z + (TAG_SIZE_M / 2)])
        self.landmarks = [self.l0,self.l1,self.l2,self.l3]
        self.camera_offset = Pose3(Rot3(), np.array([0, 0, 1]))
        self.calib = Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, -0.2, 0.1, 0.0, 0.0)

        # initialize
        self.step(0)

    def step(self, dt_s: float) -> None:
        """set all the state according to the supplied time"""
        self.time_s += dt_s
        self.gt_x = PATH_CENTER_X_M + PATH_RADIUS_M * math.cos(
            2 * math.pi * self.time_s / PATH_PERIOD_S
        )
        self.gt_y = PATH_CENTER_Y_M + PATH_RADIUS_M * math.sin(
            2 * math.pi * self.time_s / PATH_PERIOD_S
        )
        self.gt_theta = PAN_SCALE_RAD * math.sin(
            2 * math.pi * self.time_s / PAN_PERIOD_S
        )

        # find the wheel positions
        new_wpi_pose = Pose2d(self.gt_x, self.gt_y, self.gt_theta)
        twist = self.wpi_pose.log(new_wpi_pose)
        self.wpi_pose = new_wpi_pose
        self.positions = self.kinematics.to_swerve_module_positions(
            self.positions, twist
        )

        robot_pose = Pose2(self.gt_x, self.gt_y, self.gt_theta)

        # lower left
        p0 = self.px(
            self.l0,
            robot_pose,
            self.camera_offset,
            self.calib,
        )
        # lower right
        p1 = self.px(
            self.l1,
            robot_pose,
            self.camera_offset,
            self.calib,
        )
        # upper right
        p2 = self.px(
            self.l2,
            robot_pose,
            self.camera_offset,
            self.calib,
        )
        # upper left
        p3 = self.px(
            self.l3,
            robot_pose,
            self.camera_offset,
            self.calib,
        )
        self.gt_pixels = [p0, p1, p2, p3]

    def px(  # type: ignore
        self,
        landmark: np.ndarray,
        robot_pose: Pose2,
        camera_offset: Pose3,
        calib: Cal3DS2,
    ) -> Point2:
        """robot_pose and camera_offset are x-forward, z-up"""
        # ctor a pose3 with x,y,yaw, x-forward, z-up
        offset_pose = Pose3(robot_pose).compose(camera_offset)  # type: ignore
        # print("offset pose ", offset_pose)
        camera_pose = offset_pose.compose(CAM_COORD)  # type: ignore
        # camera constructor expects z-forward y-down
        # print("camera pose ", camera_pose)
        # print("landmark ", landmark)
        camera = PinholeCameraCal3DS2(camera_pose, calib)
        return camera.project(landmark)  # type: ignore
