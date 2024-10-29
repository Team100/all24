# pylint: disable=C0103,E0611,E1101

import unittest
import gtsam

import app.pose_estimator.gyro as gyro


class GyroTest(unittest.TestCase):
    def test_gyro(self) -> None:
        