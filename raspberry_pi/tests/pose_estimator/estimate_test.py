"""Test the estimator alone."""

# pylint: disable=C0301,E0611,E1101,R0903

import unittest

import gtsam
from gtsam.symbol_shorthand import X
from wpimath.geometry import Pose2d

from app.pose_estimator.estimate import Estimate


class EstimateAccelerometerTest(unittest.TestCase):
    def test_no_factors(self) -> None:
        """Nothing bad happens if you don't have any factors."""
        est = Estimate()
        est.init(Pose2d())
        time_us: int = 1
        est.add_state(time_us, gtsam.Pose2())
        est.update()
        self.assertEqual(2, est.result.size())
        p0: gtsam.Pose2 = est.result.atPose2(X(0))
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        p1: gtsam.Pose2 = est.result.atPose2(X(1))
        self.assertAlmostEqual(0, p1.x())
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(0, p1.theta())