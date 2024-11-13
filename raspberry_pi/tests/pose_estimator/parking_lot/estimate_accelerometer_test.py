"""Evaluate the estimator for accelerometry."""

# pylint: disable=C0301,E0611,E1101,R0903

import unittest

import gtsam
import numpy as np

from gtsam import noiseModel  # type:ignore

from gtsam.symbol_shorthand import X  # type:ignore

from app.pose_estimator.parking_lot.parking_lot import ParkingLot

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


class EstimateAccelerometerTest(unittest.TestCase):
    def test_accelerometer_0(self) -> None:
        est = ParkingLot()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        est.add_state(20000, gtsam.Pose2(0.02, 0, 0))
        est.add_state(40000, gtsam.Pose2(0.04, 0, 0))
        est.accelerometer(0, 20000, 40000, 0, 0)
        est.update()
        self.assertEqual(3, est.result_size())
        p0: gtsam.Pose2 = est.mean_pose2(X(0))
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        p1: gtsam.Pose2 = est.mean_pose2(X(20000))
        self.assertAlmostEqual(0.02, p1.x())
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(0, p1.theta())
        p2: gtsam.Pose2 = est.mean_pose2(X(40000))
        self.assertAlmostEqual(0.04, p2.x())
        self.assertAlmostEqual(0, p2.y())
        self.assertAlmostEqual(0, p2.theta())

    def test_accelerometer_1(self) -> None:
        est = ParkingLot()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        est.add_state(20000, gtsam.Pose2(0.02, 0, 0))
        est.add_state(40000, gtsam.Pose2(0.04, 0, 0))
        # non-zero accel
        est.accelerometer(0, 20000, 40000, 1, 0)
        est.update()
        self.assertEqual(3, est.result_size())
        p0: gtsam.Pose2 = est.mean_pose2(X(0))
        # this state has a prior so it is relatively immobile
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        p1: gtsam.Pose2 = est.mean_pose2(X(20000))
        # accel nudges this state back a bit
        self.assertAlmostEqual(0.0198, p1.x(), 3)
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(0, p1.theta())
        # accel nudges this state forward a bit
        p2: gtsam.Pose2 = est.mean_pose2(X(40000))
        self.assertAlmostEqual(0.04008, p2.x(), 5)
        self.assertAlmostEqual(0, p2.y())
        self.assertAlmostEqual(0, p2.theta())
