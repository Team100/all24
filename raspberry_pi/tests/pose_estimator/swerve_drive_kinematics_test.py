import math
import unittest

from wpimath.geometry import Rotation2d, Translation2d

from app.pose_estimator.swerve_drive_kinematics import SwerveDriveKinematics100
from app.pose_estimator.swerve_module_delta import (SwerveModuleDelta,
                                                    SwerveModuleDeltas)
from app.pose_estimator.swerve_module_position import OptionalRotation2d


class SwerveDriveKinematics100Test(unittest.TestCase):
    def test_inverse(self) -> None:
        """Same as java"""
        k = SwerveDriveKinematics100(
            [
                Translation2d(0.5, 0.5),
                Translation2d(0.5, -0.5),
                Translation2d(-0.5, 0.5),
                Translation2d(-0.5, -0.5),
            ]
        )
        self.assertEqual(8, k.inverse_kinematics.shape[0])
        self.assertEqual(3, k.inverse_kinematics.shape[1])

        self.assertAlmostEqual(1, k.inverse_kinematics[0, 0])
        self.assertAlmostEqual(0, k.inverse_kinematics[0, 1])
        self.assertAlmostEqual(-0.5, k.inverse_kinematics[0, 2])
        self.assertAlmostEqual(0, k.inverse_kinematics[1, 0])
        self.assertAlmostEqual(1, k.inverse_kinematics[1, 1])
        self.assertAlmostEqual(0.5, k.inverse_kinematics[1, 2])
        self.assertAlmostEqual(1, k.inverse_kinematics[2, 0])
        self.assertAlmostEqual(0, k.inverse_kinematics[2, 1])
        self.assertAlmostEqual(0.5, k.inverse_kinematics[2, 2])
        self.assertAlmostEqual(0, k.inverse_kinematics[3, 0])
        self.assertAlmostEqual(1, k.inverse_kinematics[3, 1])
        self.assertAlmostEqual(0.5, k.inverse_kinematics[3, 2])

    def test_forward(self) -> None:
        """Same as java"""
        k = SwerveDriveKinematics100(
            [
                Translation2d(0.5, 0.5),
                Translation2d(0.5, -0.5),
                Translation2d(-0.5, 0.5),
                Translation2d(-0.5, -0.5),
            ]
        )
        self.assertEqual(3, k.forward_kinematics.shape[0])
        self.assertEqual(8, k.forward_kinematics.shape[1])
        self.assertAlmostEqual(0.25, k.forward_kinematics[0, 0])
        self.assertAlmostEqual(0, k.forward_kinematics[1, 0])
        self.assertAlmostEqual(-0.25, k.forward_kinematics[2, 0])
        self.assertAlmostEqual(0, k.forward_kinematics[0, 1])
        self.assertAlmostEqual(0.25, k.forward_kinematics[1, 1])
        self.assertAlmostEqual(0.25, k.forward_kinematics[2, 1])
        self.assertAlmostEqual(0.25, k.forward_kinematics[0, 2])
        self.assertAlmostEqual(0, k.forward_kinematics[1, 2])
        self.assertAlmostEqual(0.25, k.forward_kinematics[2, 2])
        self.assertAlmostEqual(0, k.forward_kinematics[0, 3])
        self.assertAlmostEqual(0.25, k.forward_kinematics[1, 3])
        self.assertAlmostEqual(0.25, k.forward_kinematics[2, 3])

    def test_twist_straight(self) -> None:
        kinematics = SwerveDriveKinematics100(
            [
                Translation2d(0.5, 0.5),
                Translation2d(0.5, -0.5),
                Translation2d(-0.5, 0.5),
                Translation2d(-0.5, -0.5),
            ]
        )
        # 0.1m straight ahead, all same.
        twist = kinematics.to_twist_2d(
            SwerveModuleDeltas(
                SwerveModuleDelta(
                    0.1, OptionalRotation2d(True, Rotation2d.fromDegrees(0))
                ),
                SwerveModuleDelta(
                    0.1, OptionalRotation2d(True, Rotation2d.fromDegrees(0))
                ),
                SwerveModuleDelta(
                    0.1, OptionalRotation2d(True, Rotation2d.fromDegrees(0))
                ),
                SwerveModuleDelta(
                    0.1, OptionalRotation2d(True, Rotation2d.fromDegrees(0))
                ),
            )
        )

        self.assertAlmostEqual(0.1, twist.dx, 9)
        self.assertAlmostEqual(0, twist.dy, 9)
        self.assertAlmostEqual(0, twist.dtheta, 9)

    def test_twist_spin(self) -> None:
        kinematics = SwerveDriveKinematics100(
            [
                Translation2d(0.5, 0.5),
                Translation2d(0.5, -0.5),
                Translation2d(-0.5, 0.5),
                Translation2d(-0.5, -0.5),
            ]
        )

        twist = kinematics.to_twist_2d(
            SwerveModuleDeltas(
                SwerveModuleDelta(
                    0.1, OptionalRotation2d(True, Rotation2d.fromDegrees(135))
                ),
                SwerveModuleDelta(
                    0.1, OptionalRotation2d(True, Rotation2d.fromDegrees(45))
                ),
                SwerveModuleDelta(
                    0.1, OptionalRotation2d(True, Rotation2d.fromDegrees(-135))
                ),
                SwerveModuleDelta(
                    0.1, OptionalRotation2d(True, Rotation2d.fromDegrees(-45))
                ),
            )
        )

        self.assertAlmostEqual(0, twist.dx, 9)
        self.assertAlmostEqual(0, twist.dy, 9)
        self.assertAlmostEqual(0.141, twist.dtheta, 3)

    def test_straight_line_forward_kinematics_with_deltas(self) -> None:
        m_fl = Translation2d(12, 12)
        m_fr = Translation2d(12, -12)
        m_bl = Translation2d(-12, 12)
        m_br = Translation2d(-12, -12)

        m_kinematics = SwerveDriveKinematics100([m_fl, m_fr, m_bl, m_br])

        # test forward kinematics going in a straight line
        delta = SwerveModuleDelta(
            5.0, OptionalRotation2d(True, Rotation2d.fromDegrees(0.0))
        )
        twist = m_kinematics.to_twist_2d(SwerveModuleDeltas(
            delta, delta, delta, delta))

        self.assertAlmostEqual(5.0, twist.dx, 9)
        self.assertAlmostEqual(0.0, twist.dy, 9)
        self.assertAlmostEqual(0.0, twist.dtheta, 9)

    def test_straight_strafe_forward_kinematics_with_deltas(self) -> None:
        m_fl = Translation2d(12, 12)
        m_fr = Translation2d(12, -12)
        m_bl = Translation2d(-12, 12)
        m_br = Translation2d(-12, -12)

        m_kinematics = SwerveDriveKinematics100([m_fl, m_fr, m_bl, m_br])

        delta = SwerveModuleDelta(
            5.0, OptionalRotation2d(True, Rotation2d.fromDegrees(90.0))
        )
        twist = m_kinematics.to_twist_2d(
            SwerveModuleDeltas(delta, delta, delta, delta))

        self.assertAlmostEqual(0.0, twist.dx, 9)
        self.assertAlmostEqual(5.0, twist.dy, 9)
        self.assertAlmostEqual(0.0, twist.dtheta, 9)

    def test_turn_in_place_forward_kinematics_with_deltas(self) -> None:
        m_fl = Translation2d(12, 12)
        m_fr = Translation2d(12, -12)
        m_bl = Translation2d(-12, 12)
        m_br = Translation2d(-12, -12)

        m_kinematics = SwerveDriveKinematics100([m_fl, m_fr, m_bl, m_br])

        fl_delta = SwerveModuleDelta(
            106.629, OptionalRotation2d(True, Rotation2d.fromDegrees(135))
        )
        fr_delta = SwerveModuleDelta(
            106.629, OptionalRotation2d(True, Rotation2d.fromDegrees(45))
        )
        bl_delta = SwerveModuleDelta(
            106.629, OptionalRotation2d(True, Rotation2d.fromDegrees(-135))
        )
        br_delta = SwerveModuleDelta(
            106.629, OptionalRotation2d(True, Rotation2d.fromDegrees(-45))
        )

        twist = m_kinematics.to_twist_2d(
            SwerveModuleDeltas(fl_delta, fr_delta, bl_delta, br_delta))

        self.assertAlmostEqual(0.0, twist.dx, 9)
        self.assertAlmostEqual(0.0, twist.dy, 9)
        self.assertAlmostEqual(2 * math.pi, twist.dtheta, 1)
