# pylint: disable=C0301,E0611,R0903

import math
import unittest

from gtsam import Cal3DS2, Pose2, Pose3, Point2, Point3, Rot3
import numpy as np

# this works with runtests.py but not the little triangle up there
from tests.pose_estimator.simulator import Simulator


class SimulatorTest(unittest.TestCase):
    def test_simple(self) -> None:
        sim = Simulator()
        self.assertAlmostEqual(2, sim.gt_x)
        self.assertAlmostEqual(0, sim.gt_y)
        self.assertAlmostEqual(0, sim.gt_theta)
        # lower left
        self.assertAlmostEqual(192, sim.pixels[0][0], 0)
        self.assertAlmostEqual(208, sim.pixels[0][1], 0)
        # lower right
        self.assertAlmostEqual(208, sim.pixels[1][0], 0)
        self.assertAlmostEqual(208, sim.pixels[1][1], 0)
        # upper right
        self.assertAlmostEqual(208, sim.pixels[2][0], 0)
        self.assertAlmostEqual(192, sim.pixels[2][1], 0)
        # upper left
        self.assertAlmostEqual(192, sim.pixels[3][0], 0)
        self.assertAlmostEqual(192, sim.pixels[3][1], 0)

        sim.step(math.pi / 2)
        self.assertAlmostEqual(1, sim.gt_x)
        self.assertAlmostEqual(1, sim.gt_y)
        self.assertAlmostEqual(-0.5, sim.gt_theta)
        sim.step(math.pi / 2)
        self.assertAlmostEqual(0, sim.gt_x)
        self.assertAlmostEqual(0, sim.gt_y)
        self.assertAlmostEqual(0, sim.gt_theta)
        sim.step(math.pi / 2)
        self.assertAlmostEqual(1, sim.gt_x)
        self.assertAlmostEqual(-1, sim.gt_y)
        self.assertAlmostEqual(0.5, sim.gt_theta)

    def test_camera(self) -> None:
        sim = Simulator()
        # this is the lower right corner
        landmark: Point3 = Point3(4, -(0.1651 / 2), 1 - (0.1651 / 2))
        robot_pose: Pose2 = Pose2(2, 0, 0)
        camera_offset: Pose3 = Pose3(Rot3(), np.array([0, 0, 1]))
        calib: Cal3DS2 = Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, -0.2, 0.1, 0.0, 0.0)
        px: Point2 = sim.px(landmark, robot_pose, camera_offset, calib)
        # pixel should be in the lower right quadrant
        # remember x+right, y+down
        self.assertAlmostEqual(208, px[0], 0)
        self.assertAlmostEqual(208, px[1], 0)
