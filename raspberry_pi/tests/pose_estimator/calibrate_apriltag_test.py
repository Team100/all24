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
        est = Calibrate(0.1)
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
        print(p0)
        self.assertAlmostEqual(0, p0.x(), 3)
        self.assertAlmostEqual(0, p0.y(), 3)
        self.assertAlmostEqual(0, p0.theta(), 3)
        s0 = est.sigma(X(0))
        print(s0)
        self.assertTrue(np.allclose(np.array([0.296, 0.279, 0.099]), s0, atol=0.001))

        c0: gtsam.Pose3 = est.mean_pose3(C(0))
        self.assertAlmostEqual(0, c0.x(), 3)
        self.assertAlmostEqual(0, c0.y(), 3)
        self.assertAlmostEqual(0.6, c0.z(), 3)
        # these rotations match the offset
        # which is z-fwd remember
        rotVec = c0.rotation().xyz()
        print("rotVec", rotVec)
        self.assertTrue(np.allclose(np.array([-1.571, 0, -1.571]), rotVec, atol=0.001))
        sc0 = est.sigma(C(0))
        print("sc0", sc0)
        self.assertTrue(
            np.allclose(
                np.array([0.163, 0.191, 0.094, 0.194, 0.164, 0.199]), sc0, atol=0.001
            )
        )

        k0: gtsam.Cal3DS2 = est.mean_cal3DS2(K(0))
        print(k0)
        self.assertAlmostEqual(180, k0.fx(), 3)
        self.assertAlmostEqual(180, k0.fy(), 3)
        self.assertAlmostEqual(400, k0.px(), 3)
        self.assertAlmostEqual(300, k0.py(), 3)
        self.assertAlmostEqual(0, k0.k1(), 3)
        self.assertAlmostEqual(0, k0.k2(), 3)
        # these are not wrapped
        # self.assertAlmostEqual(0, k0.p1(), 3)
        # self.assertAlmostEqual(0, k0.p2(), 3)

        # not meaningful i think
        sk0 = est.sigma(K(0))
        print(sk0)
        self.assertTrue(
            np.allclose(np.array([10, 10, 0, 0, 0, 0.8, 0.8, 0, 0]), sk0, atol=0.1)
        )
