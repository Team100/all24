"""Test the estimator alone."""

# pylint: disable=C0301,E0611,E1101,R0903

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

        time_us: int = 1
        est.add_state(time_us, gtsam.Pose2())
        est.update()
        self.assertEqual(4, est.result.size())
        p0: gtsam.Pose2 = est.result.atPose2(X(0))
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        p1: gtsam.Pose2 = est.result.atPose2(X(1))
        self.assertAlmostEqual(0, p1.x())
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(0, p1.theta())
