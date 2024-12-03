import unittest

import random

from wpimath.geometry import Rotation2d, Translation2d, Twist2d

from app.util.drive_util import DriveUtil
from app.kinodynamics.swerve_drive_kinematics import SwerveDriveKinematics100
from app.kinodynamics.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
    SwerveModulePositions,
)

# pylint: disable=C0200,R0903


class DriveUtilTest(unittest.TestCase):
    def test_round_trip_module_deltas(self) -> None:
        k = SwerveDriveKinematics100(
            [
                Translation2d(0.5, 0.5),
                Translation2d(0.5, -0.5),
                Translation2d(-0.5, 0.5),
                Translation2d(-0.5, -0.5),
            ]
        )
        self.assertEqual(4, k.num_modules)

        # straight diagonal path
        t = Twist2d(1, 1, 0)
        p = k.to_swerve_module_delta(t)
        t2 = k.to_twist_2d(p)
        self.assertAlmostEqual(t.dx, t2.dx)
        self.assertAlmostEqual(t.dy, t2.dy)
        self.assertAlmostEqual(t.dtheta, t2.dtheta)

        # turning and moving
        t = Twist2d(1, 1, 1)
        p = k.to_swerve_module_delta(t)
        t2 = k.to_twist_2d(p)
        self.assertAlmostEqual(t.dx, t2.dx)
        self.assertAlmostEqual(t.dy, t2.dy)
        self.assertAlmostEqual(t.dtheta, t2.dtheta)

        for _ in range(500):
            # inverse always works
            t = Twist2d(random.random(), random.random(), random.random())
            p = k.to_swerve_module_delta(t)
            t2 = k.to_twist_2d(p)
            self.assertAlmostEqual(t.dx, t2.dx)
            self.assertAlmostEqual(t.dy, t2.dy)
            self.assertAlmostEqual(t.dtheta, t2.dtheta)

    def test_delta(self) -> None:
        start = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )
        end = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )
        delta = DriveUtil.module_position_delta(start, end)
        self.assertAlmostEqual(0, delta.front_left.distance_m)
        self.assertAlmostEqual(0, delta.front_left.angle.value.radians())

    def test_delta2(self) -> None:
        start = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )
        end = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(1))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(1))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(1))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(1))),
        )
        # there is no interpolation, we use the ending value.
        delta = DriveUtil.module_position_delta(start, end)
        self.assertAlmostEqual(0, delta.front_left.distance_m)
        self.assertAlmostEqual(1, delta.front_left.angle.value.radians())

    def test_delta3(self) -> None:
        start = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )
        end = SwerveModulePositions(
            SwerveModulePosition100(1, OptionalRotation2d(True, Rotation2d(1))),
            SwerveModulePosition100(1, OptionalRotation2d(True, Rotation2d(1))),
            SwerveModulePosition100(1, OptionalRotation2d(True, Rotation2d(1))),
            SwerveModulePosition100(1, OptionalRotation2d(True, Rotation2d(1))),
        )
        # there is no interpolation, we use the ending value.
        delta = DriveUtil.module_position_delta(start, end)
        self.assertAlmostEqual(1, delta.front_left.distance_m)
        self.assertAlmostEqual(1, delta.front_left.angle.value.radians())
