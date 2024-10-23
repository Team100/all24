""" Evaluate the estimation model. """

# pylint: disable=C0301,E0611,R0903

import math
import unittest

from gtsam import Cal3DS2, Pose2, Pose3, Point2, Point3, Rot3
import numpy as np

# this works with runtests.py but not the little triangle up there
from app.pose_estimator.estimate import Estimate
from tests.pose_estimator.simulator import Simulator


class EstimateTest(unittest.TestCase):
    def test_eval(self) -> None:
        sim = Simulator()
        est = Estimate()