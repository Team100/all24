import unittest

from wpimath.geometry import Translation2d

from app.pose_estimator.swerve_drive_kinematics import SwerveDriveKinematics100


class SwerveDriveKinematics100Test(unittest.TestCase):
    def test_inverse(self) -> None:
        """ Same as java"""
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
