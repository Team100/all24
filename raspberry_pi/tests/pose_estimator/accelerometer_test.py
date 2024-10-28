# pylint: disable=C0103,E0611,E1101

import unittest
import gtsam

import app.pose_estimator.accelerometer as accelerometer


class AccelerometerTest(unittest.TestCase):
    def test_accel_factor(self) -> None:
        f: gtsam.NonlinearFactor = accelerometer.factor()
        v = gtsam.Values()
        p0 = gtsam.Pose2()
        p1 = gtsam.Pose2()
        v.insert(0, p0)
        v.insert(1, p1)