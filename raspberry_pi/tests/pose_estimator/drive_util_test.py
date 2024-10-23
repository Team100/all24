import unittest

from wpimath.geometry import Rotation2d

from app.pose_estimator.drive_util import DriveUtil
from app.pose_estimator.swerve_module_position import (OptionalRotation2d,
                                                       SwerveModulePosition100)

# pylint: disable=C0200,R0903



class DriveUtilTest(unittest.TestCase):
    def test_delta(self) -> None:
        start = [
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        ]
        end = [
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        ]
        lerp = DriveUtil.module_position_delta(start, end)
        self.assertAlmostEqual(0, lerp[0].distance_m)
        self.assertAlmostEqual(0, lerp[0].angle.value.radians())

    def test_delta2(self) -> None:
        start = [
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        ]
        end = [
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(1))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(1))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(1))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(1))),
        ]
        lerp = DriveUtil.module_position_delta(start, end)
        self.assertAlmostEqual(0, lerp[0].distance_m)
        self.assertAlmostEqual(0.5, lerp[0].angle.value.radians())


    def test_delta3(self) -> None:
        start = [
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        ]
        end = [
            SwerveModulePosition100(1, OptionalRotation2d(True, Rotation2d(1))),
            SwerveModulePosition100(1, OptionalRotation2d(True, Rotation2d(1))),
            SwerveModulePosition100(1, OptionalRotation2d(True, Rotation2d(1))),
            SwerveModulePosition100(1, OptionalRotation2d(True, Rotation2d(1))),
        ]
        lerp = DriveUtil.module_position_delta(start, end)
        self.assertAlmostEqual(1, lerp[0].distance_m)
        self.assertAlmostEqual(0.5, lerp[0].angle.value.radians())