"""Test the estimator alone."""

# pylint: disable=C0301,E0611,E1101,R0903

from typing import cast
import unittest

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore
from wpimath.geometry import Pose2d, Rotation2d

from app.pose_estimator.estimate import Estimate
from app.kinodynamics.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
    SwerveModulePositions,
)

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


class EstimateAccelerometerTest(unittest.TestCase):
    def test_no_factors(self) -> None:
        """Nothing bad happens if you don't have any factors."""
        est = Estimate(0.1)
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        est.add_state(1, gtsam.Pose2())
        est.update()
        self.assertEqual(2, est.result_size())

        p0: gtsam.Pose2 = est.mean_pose2(X(0))

        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())

        m = est.marginal_covariance()
        c0 = m.marginalCovariance(X(0))
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.09, 0.00, 0.00],
                        [0.00, 0.09, 0.00],
                        [0.00, 0.00, 0.01],
                    ]
                ),
                c0,
                atol=0.001,
            )
        )
        s0 = est.sigma_pose2(X(0))
        print(s0)
        # the prior sigma
        self.assertTrue(
            np.allclose(
                np.array(
                    [0.3, 0.3, 0.1],
                ),
                s0,
                atol=0.001,
            )
        )

        p1: gtsam.Pose2 = est.mean_pose2(X(1))
        self.assertAlmostEqual(0, p1.x())
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(0, p1.theta())

    def test_result(self) -> None:
        """Just one state and one prior factor."""
        est = Estimate(0.1)
        est.init()

        est.add_state(0, gtsam.Pose2(0, 0, 0))
        # noise model is expressed as sigma
        est.prior(
            0,
            gtsam.Pose2(0, 0, 0),
            noiseModel.Diagonal.Sigmas(np.array([1, 2, 3])),
        )
        est.update()

        results = est.get_result()
        self.assertIsNotNone(results)
        results = cast(tuple[int, gtsam.Pose2, np.ndarray], results)
        t: int = results[0]
        p: gtsam.Pose2 = results[1]
        cov: np.ndarray = results[2]

        self.assertAlmostEqual(0, t)

        self.assertAlmostEqual(0, p.x())
        self.assertAlmostEqual(0, p.y())
        self.assertAlmostEqual(0, p.theta())

        print(cov)
        # variance is sigma squared
        self.assertTrue(
            np.allclose(
                cov,
                np.array(
                    [
                        [1, 0, 0],
                        [0, 4, 0],
                        [0, 0, 9],
                    ],
                ),
            )
        )

    def test_multiple_priors(self) -> None:
        """One state but two different priors: the tighter prior "wins"."""
        est = Estimate(0.1)
        est.init()

        est.add_state(0, gtsam.Pose2(0, 0, 0))
        # a very loose prior
        est.prior(
            0,
            gtsam.Pose2(0, 0, 0),
            noiseModel.Diagonal.Sigmas(np.array([10, 10, 10])),
        )

        # a much tighter prior
        est.prior(
            0,
            gtsam.Pose2(0, 0, 0),
            noiseModel.Diagonal.Sigmas(np.array([1, 1, 1])),
        )

        est.update()

        results = est.get_result()
        self.assertIsNotNone(results)
        results = cast(tuple[int, gtsam.Pose2, np.ndarray], results)
        t: int = results[0]
        p: gtsam.Pose2 = results[1]
        cov: np.ndarray = results[2]

        self.assertAlmostEqual(0, t)

        self.assertAlmostEqual(0, p.x())
        self.assertAlmostEqual(0, p.y())
        self.assertAlmostEqual(0, p.theta())

        print(cov)
        # note the resulting variance is *better* than either
        # prior alone, because they're assumed to be independent.
        self.assertTrue(
            np.allclose(
                cov,
                np.array(
                    [
                        [0.99, 0, 0],
                        [0, 0.99, 0],
                        [0, 0, 0.99],
                    ],
                ),
                atol=0.01,
            )
        )

    def test_marginals(self) -> None:
        est = Estimate(0.1)
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        # noise model is expressed as sigma
        noise = noiseModel.Diagonal.Sigmas(np.array([1, 2, 3]))
        est.prior(0, prior_mean, noise)

        est.update()

        cov: np.ndarray = est.marginals()
        print(cov)
        # variance is sigma squared
        self.assertTrue(
            np.allclose(
                cov,
                np.array(
                    [
                        [1, 0, 0],
                        [0, 4, 0],
                        [0, 0, 9],
                    ],
                ),
            )
        )

    def test_joint_marginals(self) -> None:
        """OK we're not actually going to use this."""
        est = Estimate(0.1)
        est.init()

        prior_noise = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)

        est.prior(0, prior_mean, prior_noise)

        est.add_state(1, gtsam.Pose2())

        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))

        positions = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )

        # this will just record the positions and timestamp
        est.odometry(0, positions, odometry_noise)

        positions = SwerveModulePositions(
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
        )

        # this should actually apply the between factor
        est.odometry(1, positions, odometry_noise)

        est.update()

        cov = est.joint_marginals()
        np.set_printoptions(suppress=True, precision=4)
        print(cov.fullMatrix())

    def test_extrapolate(self) -> None:
        """extrapolate the most-recent odometry."""
        est = Estimate(0.1)
        est.init()

        prior_noise = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, prior_noise)

        est.add_state(1, gtsam.Pose2())
        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))

        positions = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )

        # this will just record the positions and timestamp
        est.odometry(0, positions, odometry_noise)

        positions = SwerveModulePositions(
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
        )

        est.odometry(1, positions, odometry_noise)

        est.update()

        results = est.get_result()
        self.assertIsNotNone(results)
        results = cast(tuple[int, gtsam.Pose2, np.ndarray], results)
        t: int = results[0]
        p1: gtsam.Pose2 = results[1]
        cov: np.ndarray = results[2]

        p2 = est.extrapolate(p1)

        print("p0")
        print(prior_mean)
        print("p1")
        print(p1)
        print("extrapolated")
        print(p2)
        self.assertAlmostEqual(0.2, p2.x())
        self.assertAlmostEqual(0.0, p2.y())
        self.assertAlmostEqual(0.0, p2.theta())
