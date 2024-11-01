"""Evaluate the estimation model for apriltag measurements."""

# pylint: disable=C0103,E0611,E1101,R0913


import unittest

import gtsam
import numpy as np
from wpimath.geometry import Pose2d
from gtsam.symbol_shorthand import C, K, X  # type:ignore

from app.pose_estimator.estimate import Estimate


class EstimateAprilTagTest(unittest.TestCase):
    def test_apriltag_0(self) -> None:
        """test the calibration factor"""
        est = Estimate()
        est.init(Pose2d(0, 0, 0))
        # upper left
        landmark = np.array([1, 1, 1])
        measured = np.array([0, 0])
        est.apriltag_for_calibration(landmark, measured, 0)
        est.update()
        print(est.result)
        # there are a lot of degrees of freedom here
        # so nothing interesting is going to happen.
        self.assertEqual(3, est.result.size())
        p0: gtsam.Pose2 = est.result.atPose2(X(0))
        self.assertAlmostEqual(-0.14, p0.x(), 1)
        self.assertAlmostEqual(0, p0.y(), 1)
        self.assertAlmostEqual(0, p0.theta(), 1)
        c0: gtsam.Pose3 = est.result.atPose3(C(0))
        self.assertAlmostEqual(0, c0.x(), 1)
        k0: gtsam.Cal3DS2 = est.result.atCal3DS2(K(0))
        self.assertAlmostEqual(59.993, k0.fx(), 3)

    def test_apriltag_1(self) -> None:
        """Test the smoothing factor."""
        est = Estimate()
        est.init(Pose2d(0, 0, 0))
        # upper left
        landmark = np.array([1, 1, 1])
        measured = np.array([0, 0])
        # this uses constant offset and cal
        # TODO: specify offset and cal here
        est.apriltag_for_smoothing(
            landmark,
            measured,
            0,
            gtsam.Pose3(),
            gtsam.Cal3DS2(60.0, 60.0, 0.0, 45.0, 45.0, 0.0, 0.0, 0.0, 0.0),
        )
        est.update()
        print(est.result)
        # there are a lot of degrees of freedom here
        # so nothing interesting is going to happen.
        self.assertEqual(3, est.result.size())
        p0: gtsam.Pose2 = est.result.atPose2(X(0))
        # these are because the camera cal is ridiculously wrong
        self.assertAlmostEqual(-0.3, p0.x(), 1)
        self.assertAlmostEqual(-0.03, p0.y(), 1)
        self.assertAlmostEqual(0.03, p0.theta(), 1)
        c0: gtsam.Pose3 = est.result.atPose3(C(0))
        self.assertAlmostEqual(0, c0.x(), 1)
        k0: gtsam.Cal3DS2 = est.result.atCal3DS2(K(0))
        self.assertAlmostEqual(60, k0.fx(), 3)
