""" Evaluate the estimation model for odometry. """

# pylint: disable=C0301,E0611,E1101,R0903

import math
import unittest

import gtsam
import numpy as np

from gtsam import noiseModel  # type:ignore

from gtsam.symbol_shorthand import X  # type:ignore
from wpimath.geometry import Rotation2d

from app.pose_estimator.estimate import Estimate
from app.kinodynamics.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
    SwerveModulePositions,
)

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


class EstimateOdometryTest(unittest.TestCase):
    def test_odometry_0(self) -> None:
        # initial position at origin
        est = Estimate(0.1)
        est.init()

        # std dev of 1 cm and 0.5 deg
        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        positions = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )
        est.odometry(0, positions, odometry_noise)

        # drive straight ahead 0.1
        est.add_state(1, gtsam.Pose2())
        positions = SwerveModulePositions(
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
        )
        est.odometry(1, positions, odometry_noise)

        est.update()

        self.assertEqual(2, est.result_size())

        p0: gtsam.Pose2 = est.mean_pose2(X(0))
        self.assertAlmostEqual(0, p0.x(), 6)
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())

        m = est.marginal_covariance()
        c0 = m.marginalCovariance(X(0))
        print("c0", c0)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.045, 0.00, 0.00],
                        [0.00, 0.045, 0.00],
                        [0.00, 0.00, 0.005],
                    ]
                ),
                c0,
                atol=0.001,
            )
        )
        s0 = est.sigma_pose2(X(0))
        print(s0)
        # i don't really understand why the sigma is smaller than the prior sigma
        self.assertTrue(
            np.allclose(
                np.array(
                    [0.212, 0.212, 0.071],
                ),
                s0,
                atol=0.001,
            )
        )

        # this is the x value from above
        p1: gtsam.Pose2 = est.mean_pose2(X(1))
        self.assertAlmostEqual(0.1, p1.x(), 6)
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(0, p1.theta())

        m = est.marginal_covariance()
        c1 = m.marginalCovariance(X(1))
        print("c1", c1)
        #
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.045, 0.00, 0.00],
                        [0.00, 0.045, 0.00],
                        [0.00, 0.00, 0.005],
                    ]
                ),
                c1,
                atol=0.001,
            )
        )
        s1 = est.sigma_pose2(X(1))
        print(s1)
        # odometry is pretty accurate so the sigma is about the same
        self.assertTrue(
            np.allclose(
                np.array(
                    [0.212, 0.212, 0.071],
                ),
                s1,
                atol=0.001,
            )
        )

    def test_odometry_1(self) -> None:
        """combined rotation and translation, no wheel slip, should yield exact solution."""
        # initial position at origin, 1m wheelbase
        est = Estimate(0.1)
        est.init()

        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        positions = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )
        est.odometry(0, positions, odometry_noise)

        est.add_state(1, gtsam.Pose2())

        # This is a rotating translation that should end up rotated 90 to the left
        # and 1m behind

        positions = SwerveModulePositions(
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
        )
        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
        est.odometry(1, positions, odometry_noise)

        est.update()

        self.assertEqual(2, est.result_size())

        p0: gtsam.Pose2 = est.mean_pose2(X(0))
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())

        m = est.marginal_covariance()
        c0 = m.marginalCovariance(X(0))
        print("c0", c0)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.045, 0.00, 0.00],
                        [0.00, 0.045, 0.00],
                        [0.00, 0.00, 0.005],
                    ]
                ),
                c0,
                atol=0.001,
            )
        )
        s0 = est.sigma_pose2(X(0))
        print(s0)
        # i don't really understand why the sigma is smaller than the prior sigma
        self.assertTrue(
            np.allclose(
                np.array(
                    [0.212, 0.212, 0.071],
                ),
                s0,
                atol=0.001,
            )
        )

        # this is the x value from above
        p1: gtsam.Pose2 = est.mean_pose2(X(1))
        self.assertAlmostEqual(-1, p1.x())
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(math.pi / 2, p1.theta())

        m = est.marginal_covariance()
        c1 = m.marginalCovariance(X(1))
        print("c1", c1)
        # curve means correlating x and theta?
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.05, 0.00, -0.005],
                        [0.00, 0.045, 0.00],
                        [-0.005, 0.00, 0.005],
                    ]
                ),
                c1,
                atol=0.001,
            )
        )
        s1 = est.sigma_pose2(X(1))
        print(s1)
        # a bit more x err than the straight course
        self.assertTrue(
            np.allclose(
                np.array(
                    [0.224, 0.212, 0.071],
                ),
                s1,
                atol=0.001,
            )
        )

    def test_odometry_2(self) -> None:
        """add some wheel slip to the above case."""
        # initial position at origin, 1m wheelbase
        est = Estimate(0.1)
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        positions = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )

        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
        # this should just record the positions and timestamp
        est.odometry(0, positions, odometry_noise)

        est.add_state(1, gtsam.Pose2())
        # This is a rotating translation that should end up rotated 90 to the left
        # and 1m behind
        positions = SwerveModulePositions(
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
        )
        est.odometry(1, positions, odometry_noise)

        est.update()

        self.assertEqual(2, est.result_size())
        p0: gtsam.Pose2 = est.mean_pose2(X(0))
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())

        m = est.marginal_covariance()
        c0 = m.marginalCovariance(X(0))
        print("c0", c0)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.045, 0.00, 0.00],
                        [0.00, 0.045, 0.00],
                        [0.00, 0.00, 0.005],
                    ]
                ),
                c0,
                atol=0.001,
            )
        )
        s0 = est.sigma_pose2(X(0))
        print(s0)
        # i don't really understand why the sigma is smaller than the prior sigma
        self.assertTrue(
            np.allclose(
                np.array(
                    [0.212, 0.212, 0.071],
                ),
                s0,
                atol=0.001,
            )
        )

        p1: gtsam.Pose2 = est.mean_pose2(X(1))
        # a little extra rolling of one wheel means we went a little further in x
        self.assertAlmostEqual(-1.018, p1.x(), 2)
        # and scooted a little to the right
        self.assertAlmostEqual(-0.018, p1.y(), 2)
        # and over-rotated a little
        self.assertAlmostEqual(1.606, p1.theta(), 2)

        m = est.marginal_covariance()
        c1 = m.marginalCovariance(X(1))
        print("c1", c1)
        #
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.050, 0.000, -0.005],
                        [0.000, 0.045, 0.000],
                        [-0.005, 0.000, 0.005],
                    ]
                ),
                c1,
                atol=0.001,
            )
        )
        s1 = est.sigma_pose2(X(1))
        print(s1)
        # very slightly more error in x
        self.assertTrue(
            np.allclose(
                np.array(
                    [0.224, 0.212, 0.071],
                ),
                s1,
                atol=0.001,
            )
        )
