""" Evaluate the estimation model for odometry. """

# pylint: disable=C0301,E0611,E1101,R0903

import math
import unittest

import gtsam
import numpy as np

from gtsam import noiseModel  # type:ignore

from gtsam.symbol_shorthand import X # type:ignore
from wpimath.geometry import Rotation2d

from app.pose_estimator.estimate import Estimate
from app.pose_estimator.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
)

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))



class EstimateOdometryTest(unittest.TestCase):
    def test_odometry_0(self) -> None:
        # initial position at origin
        est = Estimate()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        est.add_state(1, gtsam.Pose2())

        # drive straight ahead 0.1
        positions = [
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
        ]
        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))

        est.odometry(0, 1, positions, odometry_noise)
        est.update()
        print(est.result)
        self.assertEqual(4, est.result.size())
        p0: gtsam.Pose2 = est.result.atPose2(X(0))
        self.assertAlmostEqual(0, p0.x(), 6)
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        p1: gtsam.Pose2 = est.result.atPose2(X(1))
        # this is the x value from above
        self.assertAlmostEqual(0.1, p1.x(), 6)
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(0, p1.theta())

    def test_odometry_1(self) -> None:
        """combined rotation and translation, no wheel slip, should yield exact solution."""
        # initial position at origin, 1m wheelbase
        est = Estimate()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        est.add_state(1, gtsam.Pose2())

        # This is a rotating translation that should end up rotated 90 to the left
        # and 1m behind
        # array order:
        #
        # frontLeft
        # frontRight
        # rearLeft
        # rearRight

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
        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
        est.odometry(0, 1, positions, odometry_noise)
        est.update()
        print(est.result)
        self.assertEqual(4, est.result.size())
        p0: gtsam.Pose2 = est.result.atPose2(X(0))
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        p1: gtsam.Pose2 = est.result.atPose2(X(1))
        # this is the x value from above
        self.assertAlmostEqual(-1, p1.x())
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(math.pi / 2, p1.theta())

    def test_odometry_2(self) -> None:
        """add some wheel slip to the above case."""
        # initial position at origin, 1m wheelbase
        est = Estimate()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        est.add_state(1, gtsam.Pose2())
        # This is a rotating translation that should end up rotated 90 to the left
        # and 1m behind
        # array order:
        #
        # frontLeft
        # frontRight
        # rearLeft
        # rearRight
        positions = [
            SwerveModulePosition100(
                math.sqrt(2) * math.pi / 2 + 0.1,  # inconsistency here
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
        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
        est.odometry(0, 1, positions, odometry_noise)
        est.update()
        print(est.result)
        self.assertEqual(4, est.result.size())
        p0: gtsam.Pose2 = est.result.atPose2(X(0))
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        p1: gtsam.Pose2 = est.result.atPose2(X(1))
        # a little extra rolling of one wheel means we went a little further in x
        self.assertAlmostEqual(-1.018, p1.x(), 2)
        # and scooted a little to the right
        self.assertAlmostEqual(-0.018, p1.y(), 2)
        # and over-rotated a little
        self.assertAlmostEqual(1.606, p1.theta(), 2)
