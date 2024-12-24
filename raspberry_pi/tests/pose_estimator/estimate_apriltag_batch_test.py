"""Evaluate the estimation model for apriltag measurements."""

# pylint: disable=C0103,E0611,E1101,R0913


import unittest

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore

from app.pose_estimator.estimate import Estimate

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


class EstimateAprilTagTest(unittest.TestCase):

    def test_apriltag_1(self) -> None:
        """Test the smoothing factor."""
        est = Estimate(0.1)
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        # upper left, this is from apriltag_calibrate_test.
        landmarks = [np.array([1, 0, 0]), np.array([1, 1, 1])]
        measured = np.array([200, 200, 0, 0])

        est.apriltag_for_smoothing_batch(
            landmarks,
            measured,
            0,
            gtsam.Pose3(
                gtsam.Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
                gtsam.Point3(0, 0, 0),
            ),
            gtsam.Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, -0.2, 0.1),
        )
        est.update()
        self.assertEqual(1, est.result_size())
        p0: gtsam.Pose2 = est.mean_pose2(X(0))
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        m = est.marginal_covariance()
        c0 = m.marginalCovariance(X(0))
        # maybe inverse correlation between y and theta
        # illustrates "aiming" as you move side to side?
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.008, -0.008, 0.008],
                        [-0.008, 0.008, -0.008],
                        [0.008, -0.008, 0.008],
                    ]
                ),
                c0,
                atol=0.001,
            )
        )
        s0 = est.sigma_pose2(X(0))
        # with one sight the resulting sigmas are tiny
        self.assertTrue(
            np.allclose(
                np.array(
                    [0.09, 0.09, 0.09],
                ),
                s0,
                atol=0.01,
            )
        )
