# pylint: disable=C0301,E0611,E1101,R0903,R0914

import math
import time
import unittest

import gtsam
import numpy as np
from gtsam import Cal3DS2, Point2, Point3, Pose2, Pose3, Rot3 # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore
from wpimath.geometry import Pose2d

from app.pose_estimator.estimate import Estimate
from tests.pose_estimator.simulator import Simulator

actually_print = False

class EstimateSimulateTest(unittest.TestCase):
    def test_simple(self) -> None:
        sim = Simulator()
        est = Estimate()
        est.init(sim.wpi_pose)

        print()
        for i in range(1, 100):
            t0 = time.time_ns()
            t0_us = 20000 * (i-1)
            t1_us = 20000 * i
            est.add_state(t1_us, gtsam.Pose2())
            est.odometry(t0_us, t1_us, sim.positions)
            est.update()
            t1 = time.time_ns()
            et = t1 - t0
            if actually_print:
                print(f"{et/1e9} {est.result.size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est.result.atPose2(X(t1_us))
            est_x = p.x()
            est_y = p.y()
            est_theta = p.theta()

            print(
                f"{t:7.4f}, {gt_x:7.4f}, {gt_y:7.4f}, {gt_theta:7.4f}, {est_x:7.4f}, {est_y:7.4f}, {est_theta:7.4f}"
            )

            sim.step(0.02)
