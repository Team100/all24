"""Test the estimator alone."""

# pylint: disable=C0301,E0611,E1101,R0903

from typing import cast
import unittest

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore
from wpimath.geometry import Pose2d

from app.pose_estimator.estimate import Estimate

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


class EstimateAccelerometerTest(unittest.TestCase):
    def test_no_factors(self) -> None:
        """Nothing bad happens if you don't have any factors."""
        est = Estimate()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        est.add_state(1, gtsam.Pose2())
        est.update()
        self.assertEqual(4, est.result.size())

        p0: gtsam.Pose2 = est.result.atPose2(X(0))
        p1: gtsam.Pose2 = est.result.atPose2(X(1))

        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())

        self.assertAlmostEqual(0, p1.x())
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(0, p1.theta())

    def test_result(self) -> None:
        est = Estimate()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        # noise model is expressed as sigma
        noise = noiseModel.Diagonal.Sigmas(np.array([1, 2, 3]))
        est.prior(0, prior_mean, noise)

        est.add_state(1, gtsam.Pose2())
        est.prior(1, prior_mean, noise)
        est.update()

        results = est.get_result()
        self.assertIsNotNone(results)
        results = cast(tuple[int, gtsam.Pose2, np.ndarray], results)
        t: int = results[0]
        p: gtsam.Pose2 = results[1]
        cov:np.ndarray = results[2]

        self.assertAlmostEqual(1, t)

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


    def test_marginals(self) -> None:
        est = Estimate()
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
