# pylint: disable=C0301,E0611,R0903

import math
import unittest

import numpy as np
from gtsam import Cal3DS2, Point2, Point3, Pose2, Pose3, Rot3
from wpimath.geometry import Pose2d

from app.pose_estimator.estimate import Estimate
from tests.pose_estimator.simulator import Simulator


class EstimateSimulateTest(unittest.TestCase):
    def test_simple(self) -> None:
        sim = Simulator()
        est = Estimate()
        est.init(sim.wpi_pose)

        print()
        for i in range(1,500):
            time_us = 20000 * i
            est.odometry(time_us, sim.positions)
            est.update()
            # print(est.result)
            t = i*0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            print(f"{t:5.2f} {gt_x:5.2f} {gt_y:5.2f} {gt_theta:5.2f}")
           
            sim.step(0.02)
