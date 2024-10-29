# pylint: disable=C0103,E0611,E1101


import unittest
import gtsam

import app.pose_estimator.apriltag as apriltag

class AprilTagTest(unittest.TestCase):
    def test_apriltag(self) -> None:
        