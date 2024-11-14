"""Test the calibration model for apriltags."""

# pylint: disable=C0103,E0611,E1101,R0913

import unittest

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import C, K, X  # type:ignore

from app.pose_estimator.calibrate import Calibrate

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


class CalibrateAprilTagTest(unittest.TestCase):
    def test_apriltag_0(self) -> None:
        """Test the calibration factor.
        There's not enough data here to really converge on anything, it's just
        to test the plumbing."""
        est = Calibrate()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        # upper left
        landmark = np.array([1, 1, 1])
        measured = np.array([0, 0])
        est.apriltag_for_calibration(landmark, measured, 0)

        est.update()

        # there's a fair amount of noise here
        self.assertEqual(3, est.result_size())

        p0: gtsam.Pose2 = est.mean_pose2(X(0))
        self.assertAlmostEqual(-0.149, p0.x(), 3)
        self.assertAlmostEqual(0.031, p0.y(), 1)
        self.assertAlmostEqual(0.020, p0.theta(), 3)
        s0 = est.sigma(X(0))
        print(s0)
        self.assertTrue(np.allclose(np.array([0.221, 0.212, 0.092]), s0, atol=0.001))

        c0: gtsam.Pose3 = est.mean_pose3(C(0))
        self.assertAlmostEqual(-0.016, c0.x(), 3)
        self.assertAlmostEqual(0.004, c0.y(), 3)
        self.assertAlmostEqual(0.016, c0.z(), 3)
        self.assertAlmostEqual(0.011, c0.rotation().roll(), 3)
        self.assertAlmostEqual(-0.036, c0.rotation().pitch(), 3)
        self.assertAlmostEqual(0.020, c0.rotation().yaw(), 3)
        sc0 = est.sigma(C(0))
        print(sc0)
        self.assertTrue(
            np.allclose(
                np.array([0.089, 0.082, 0.092, 0.097, 0.097, 0.094]), sc0, atol=0.001
            )
        )

        k0: gtsam.Cal3DS2 = est.mean_cal3DS2(K(0))
        print(k0)
        self.assertAlmostEqual(59.993, k0.fx(), 3)
        self.assertAlmostEqual(59.974, k0.fy(), 3)
        self.assertAlmostEqual(45.009, k0.px(), 3)
        self.assertAlmostEqual(45.035, k0.py(), 3)
        self.assertAlmostEqual(0, k0.k()[0], 3)
        self.assertAlmostEqual(0, k0.k()[1], 3)
        self.assertAlmostEqual(0, k0.k()[2], 3)
        self.assertAlmostEqual(0, k0.k()[3], 3)

        # not meaningful i think
        sk0 = est.sigma(K(0))
        print(sk0)
        self.assertTrue(
            np.allclose(np.array([1, 1, 1, 1, 1, 0, 0, 0, 0]), sk0, atol=0.1)
        )
