# pylint: disable=C0301,E0611,R0903,W0212

import math
import unittest

import numpy as np
from gtsam import Cal3DS2, Point2, Point3, Pose2, Pose3, Rot3  # type:ignore
from wpimath.geometry import Rotation2d, Translation2d

from app.util.drive_util import DriveUtil
from app.field.field_map import FieldMap
from app.kinodynamics.swerve_drive_kinematics import SwerveDriveKinematics100
from app.kinodynamics.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
    SwerveModulePositions,
)
from tests.pose_estimator.simulation.circle_simulator import CircleSimulator


class CircleSimulatorTest(unittest.TestCase):
    def test_simple(self) -> None:
        sim = CircleSimulator(FieldMap())
        self.assertAlmostEqual(2, sim.gt_x)
        self.assertAlmostEqual(0, sim.gt_y)
        self.assertAlmostEqual(0, sim.gt_theta)
        self.assertAlmostEqual(0, sim.positions.front_left.distance_m)
        self.assertAlmostEqual(0, sim.positions.front_left.angle.value.radians())

        # lower left
        self.assertAlmostEqual(384, sim.gt_pixels[0][0], 0)
        self.assertAlmostEqual(219, sim.gt_pixels[0][1], 0)
        # lower right
        self.assertAlmostEqual(416, sim.gt_pixels[1][0], 0)
        self.assertAlmostEqual(219, sim.gt_pixels[1][1], 0)
        # upper right
        self.assertAlmostEqual(416, sim.gt_pixels[2][0], 0)
        self.assertAlmostEqual(190, sim.gt_pixels[2][1], 0)
        # upper left
        self.assertAlmostEqual(384, sim.gt_pixels[3][0], 0)
        self.assertAlmostEqual(190, sim.gt_pixels[3][1], 0)

        sim.step(math.pi / 2)
        self.assertAlmostEqual(1, sim.gt_x)
        self.assertAlmostEqual(1, sim.gt_y)
        self.assertAlmostEqual(-1, sim.gt_theta)
        sim.step(math.pi / 2)
        self.assertAlmostEqual(0, sim.gt_x)
        self.assertAlmostEqual(0, sim.gt_y)
        self.assertAlmostEqual(0, sim.gt_theta)
        sim.step(math.pi / 2)
        self.assertAlmostEqual(1, sim.gt_x)
        self.assertAlmostEqual(-1, sim.gt_y)
        self.assertAlmostEqual(1, sim.gt_theta)

    def test_camera(self) -> None:
        sim = CircleSimulator(FieldMap())
        # this is the lower right corner
        landmark = Point3(4, -(0.1651 / 2.0), 1 - (0.1651 / 2))
        robot_pose = Pose2(2, 0, 0)
        camera_offset = Pose3(
            Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
            Point3(0, 0, 1),  # type: ignore
        )
        calib = Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, -0.2, 0.1)
        px: np.ndarray = sim._px(landmark, robot_pose, camera_offset, calib)
        # pixel should be in the lower right quadrant
        # remember x+right, y+down
        self.assertAlmostEqual(208, px[0], 0)
        self.assertAlmostEqual(208, px[1], 0)

    def test_full(self) -> None:
        sim = CircleSimulator(FieldMap())
        k = SwerveDriveKinematics100(
            [
                Translation2d(0.5, 0.5),
                Translation2d(0.5, -0.5),
                Translation2d(-0.5, 0.5),
                Translation2d(-0.5, -0.5),
            ]
        )
        positions = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )
        pose = sim.wpi_pose
        print()
        for i in range(500):

            # make sure position-derived pose matches the sim pose
            deltas = DriveUtil.module_position_delta(positions, sim.positions)
            twist = k.to_twist_2d(deltas)
            new_pose = pose.exp(twist)
            pose = new_pose
            positions = sim.positions
            self.assertAlmostEqual(new_pose.x, sim.wpi_pose.x)
            self.assertAlmostEqual(new_pose.y, sim.wpi_pose.y)
            self.assertAlmostEqual(
                new_pose.rotation().radians(), sim.wpi_pose.rotation().radians()
            )

            t = i * 0.02
            x = sim.gt_x
            y = sim.gt_y
            theta = sim.gt_theta
            d0 = sim.positions.front_left.distance_m
            a0 = sim.positions.front_left.angle.value.radians()
            print(f"{t:5.2f} {x:5.2f} {y:5.2f} {theta:5.2f} {d0:5.2f} {a0:5.2f}")
            sim.step(0.02)
