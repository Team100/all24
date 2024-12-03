"""Evaluate the estimation model for binary gyro measurements"""

# pylint: disable=C0301,E0611,E1101,R0903

import unittest

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore

from app.pose_estimator.parking_lot.parking_lot import ParkingLot

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


class EstimateBinaryGyroTest(unittest.TestCase):
    def test_gyro_0(self) -> None:
        """motionless"""
        est = ParkingLot()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        est.add_state(1, gtsam.Pose2(0, 0, 0))
        est.binary_gyro(0, 1, 0)
        est.update()
        self.assertEqual(2, est.result_size())
        p0: gtsam.Pose2 = est.mean_pose2(X(0))
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        p1: gtsam.Pose2 = est.mean_pose2(X(1))
        self.assertAlmostEqual(0, p1.x())
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(0, p1.theta())
        

    def test_gyro_1(self) -> None:
        """rotating"""
        est = ParkingLot()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        est.add_state(1, gtsam.Pose2(0, 0, 1))
        est.binary_gyro(0, 1, 1)
        est.update()
        self.assertEqual(2, est.result_size())
        p0: gtsam.Pose2 = est.mean_pose2(X(0))
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        p1: gtsam.Pose2 = est.mean_pose2(X(1))
        self.assertAlmostEqual(0, p1.x())
        self.assertAlmostEqual(0, p1.y())
        # note theta matches the "between"
        self.assertAlmostEqual(1, p1.theta())
