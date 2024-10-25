""" Evaluate the estimation model. """

# pylint: disable=C0301,E0611,R0903

import math
import unittest

import gtsam
from gtsam.symbol_shorthand import X
from wpimath.geometry import Rotation2d, Pose2d

from app.pose_estimator.estimate import Estimate
from app.pose_estimator.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
)


class EstimateTest(unittest.TestCase):
    def test_eval(self) -> None:
        # initial position at origin
        est = Estimate()
        est.init(Pose2d())
        # drive straight ahead 0.1
        positions = [
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
        ]
        time_us: int = 1
        est.odometry(time_us, positions)
        est.update()
        print(est.result)
        self.assertEqual(2, est.result.size())
        p0: gtsam.Pose2 = est.result.atPose2(X(0))
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        p1: gtsam.Pose2 = est.result.atPose2(X(1))
        # this is the x value from above
        self.assertAlmostEqual(0.1, p1.x())
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(0, p1.theta())

    def test_eval2(self) -> None:
        """array order:
        *
        * frontLeft
        * frontRight
        * rearLeft
        * rearRight
        """
        # initial position at origin, 1m wheelbase
        est = Estimate()
        est.init(Pose2d())
        # This is a rotating translation that should end up rotated 90 to the left
        # and 1m behind
        positions = [
            SwerveModulePosition100(
                math.sqrt(2) * math.pi / 2,
                OptionalRotation2d(True, Rotation2d.fromDegrees(135)),
            ),
            SwerveModulePosition100(
                math.pi / 2, OptionalRotation2d(True, Rotation2d.fromDegrees(90))
            ),
            SwerveModulePosition100(
                math.pi / 2, OptionalRotation2d(True, Rotation2d.fromDegrees(180))
            ),
            SwerveModulePosition100(0.0, OptionalRotation2d(False, Rotation2d(0))),
        ]
        time_us: int = 1
        est.odometry(time_us, positions)
        est.update()
        print(est.result)
        self.assertEqual(2, est.result.size())
        p0: gtsam.Pose2 = est.result.atPose2(X(0))
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        p1: gtsam.Pose2 = est.result.atPose2(X(1))
        # this is the x value from above
        self.assertAlmostEqual(-1, p1.x())
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(math.pi / 2, p1.theta())
