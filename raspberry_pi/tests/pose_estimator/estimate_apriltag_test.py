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
        est = Estimate()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

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
        self.assertEqual(1, est.result.size())
        p0: gtsam.Pose2 = est.result.atPose2(X(0))
        # these are because the camera cal is ridiculously wrong
        self.assertAlmostEqual(-0.3, p0.x(), 1)
        self.assertAlmostEqual(-0.03, p0.y(), 1)
        self.assertAlmostEqual(0.03, p0.theta(), 1)

