# pylint: disable=C0301,E0611,R0902,R0903
# mypy: disable-error-code="import-untyped"
"""Minimal simulated telemetry generator for both smoothing and calibration.

The field is 3x3 meters
The field has one tag on the far (+x) end, 0.5 m off the floor
The robot has one camera with the same pose as the robot, except 1m high.
The robot path is a circle of radius 1m, centered (1, 0), followed at 1m/s.
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
from gtsam import Point3  # type:ignore
from gtsam import Pose2  # type:ignore
from gtsam import PinholeCameraCal3DS2, Pose3, Rot3
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from app.config.camera_config import CameraConfig
from app.config.identity import Identity
from app.field.field_map import FieldMap
from app.kinodynamics.swerve_drive_kinematics import SwerveDriveKinematics100
from app.kinodynamics.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
    SwerveModulePositions,
)

# TAG_SIZE_M: float = 0.1651
# TAG_X: float = 4
# TAG_Y: float = 0
# TAG_Z: float = 1
PATH_CENTER_X_M = 1
PATH_CENTER_Y_M = 0
PATH_RADIUS_M = 1
PATH_PERIOD_S = 2.0 * math.pi
PAN_PERIOD_S = PATH_PERIOD_S / 3
# maximum pan angle, radians
PAN_SCALE_RAD = 1.0


class CircleSimulator:
    """Starts at (2,0,0)"""

    def __init__(self, field_map: FieldMap) -> None:
        self.field_map = field_map
        # TODO: actual wheelbase etc
        self.kinematics = SwerveDriveKinematics100(
            [
                Translation2d(0.5, 0.5),
                Translation2d(0.5, -0.5),
                Translation2d(-0.5, 0.5),
                Translation2d(-0.5, -0.5),
            ]
        )
        self.positions = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )
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
        self.gt_pixels: list[np.ndarray]

        # set positions back to zero
        # TODO: more clever init
        self.positions = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )
        # constant landmark points
        # tag zero is at (3, 0, 1)
        tag = self.field_map.get(0)
        self.l0 = tag[0]
        self.l1 = tag[1]
        self.l2 = tag[2]
        self.l3 = tag[3]
        self.landmarks = [self.l0, self.l1, self.l2, self.l3]

        cam = CameraConfig(Identity.UNKNOWN)

        # the camera is 0.5m from the floor
        self.camera_offset = cam.camera_offset
        # this camera is 800x600.
        self.calib = cam.calib

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
        # print("GT POSE", new_wpi_pose)

        # lower left
        p0 = self._px(
            self.l0,
            robot_pose,
            self.camera_offset,
            self.calib,
        )
        # lower right
        p1 = self._px(
            self.l1,
            robot_pose,
            self.camera_offset,
            self.calib,
        )
        # upper right
        p2 = self._px(
            self.l2,
            robot_pose,
            self.camera_offset,
            self.calib,
        )
        # upper left
        p3 = self._px(
            self.l3,
            robot_pose,
            self.camera_offset,
            self.calib,
        )
        self.gt_pixels = [p0, p1, p2, p3]
        # TODO: concatenate these pixels here

        # omit out-of-frame tags
        all_pixels = np.concatenate(self.gt_pixels)

        # TODO: proper frame boundaries
        if (
            np.any(all_pixels[::2] < 0)
            or np.any(all_pixels[1::2] < 0)
            or np.any(all_pixels[::2] > 800)
            or np.any(all_pixels[1::2] > 600)
        ):
            self.gt_pixels = []

    def _px(  # type: ignore
        self,
        landmark: np.ndarray,
        robot_pose: Pose2,
        camera_offset: Pose3,
        calib: Cal3DS2,
    ) -> np.ndarray:
        """Project the landmark point into the camera frame and return (x, y) in pixels.
        Robot_pose and camera_offset are x-forward, z-up."""
        camera_pose = Pose3(robot_pose).compose(camera_offset)  # type: ignore
        camera = PinholeCameraCal3DS2(camera_pose, calib)
        return camera.project(landmark)  # type: ignore
