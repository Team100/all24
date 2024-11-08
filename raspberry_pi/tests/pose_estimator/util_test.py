# pylint: disable=E1101


import math
import unittest

import gtsam
import numpy as np

from app.pose_estimator.util import pose3_to_pose3d, to_cal


class UtilTest(unittest.TestCase):
    def test_pose3_to_pose3d_0(self) -> None:
        gp = gtsam.Pose3(gtsam.Rot3(), np.array([0, 0, 0]))
        wp = pose3_to_pose3d(gp)
        self.assertAlmostEqual(0, wp.rotation().X())
        self.assertAlmostEqual(0, wp.rotation().Y())
        self.assertAlmostEqual(0, wp.rotation().Z())
        self.assertAlmostEqual(0, wp.translation().X())
        self.assertAlmostEqual(0, wp.translation().Y())
        self.assertAlmostEqual(0, wp.translation().Z())

    def test_pose3_to_pose3d_1(self) -> None:
        gp = gtsam.Pose3(gtsam.Rot3(), np.array([1, 2, 3]))
        wp = pose3_to_pose3d(gp)
        self.assertAlmostEqual(0, wp.rotation().X())
        self.assertAlmostEqual(0, wp.rotation().Y())
        self.assertAlmostEqual(0, wp.rotation().Z())
        self.assertAlmostEqual(1, wp.translation().X())
        self.assertAlmostEqual(2, wp.translation().Y())
        self.assertAlmostEqual(3, wp.translation().Z())

    def test_pose3_to_pose3d_2(self) -> None:
        gp = gtsam.Pose3(gtsam.Rot3().Roll(math.pi / 2), np.array([0, 0, 0]))
        wp = pose3_to_pose3d(gp)
        self.assertAlmostEqual(math.pi / 2, wp.rotation().X())
        self.assertAlmostEqual(0, wp.rotation().Y())
        self.assertAlmostEqual(0, wp.rotation().Z())
        self.assertAlmostEqual(0, wp.translation().X())
        self.assertAlmostEqual(0, wp.translation().Y())
        self.assertAlmostEqual(0, wp.translation().Z())

    def test_pose3_to_pose3d_4(self) -> None:
        # grsam ypr is is intrinsic rotation:
        # yaw 90 degrees left, pitch 90 degrees down, roll 90 degrees right
        # the corresponding extrinsic rpy should be
        # pitch 90 degree down and that's all
        gp = gtsam.Pose3(
            gtsam.Rot3().Ypr(math.pi / 2, math.pi / 2, math.pi / 2), np.array([0, 0, 0])
        )
        wp = pose3_to_pose3d(gp)
        self.assertAlmostEqual(0, wp.rotation().X())
        self.assertAlmostEqual(math.pi / 2, wp.rotation().Y())
        self.assertAlmostEqual(0, wp.rotation().Z())
        self.assertAlmostEqual(0, wp.translation().X())
        self.assertAlmostEqual(0, wp.translation().Y())
        self.assertAlmostEqual(0, wp.translation().Z())

    def test_pose3_to_pose3d_5(self) -> None:
        # roll right and move +z, where do you end up?
        gp = gtsam.Pose3(gtsam.Rot3().Roll(math.pi / 2), np.array([0, 0, 1]))
        wp = pose3_to_pose3d(gp)
        print(wp)
        # roll is roll
        self.assertAlmostEqual(math.pi / 2, wp.rotation().X())
        self.assertAlmostEqual(0, wp.rotation().Y())
        self.assertAlmostEqual(0, wp.rotation().Z())
        # z is z, i.e. translation is applied first (or, equivalently, it's extrinsic)
        self.assertAlmostEqual(0, wp.translation().X())
        self.assertAlmostEqual(0, wp.translation().Y())
        self.assertAlmostEqual(1, wp.translation().Z())

    def test_cal(self) -> None:
        gc = gtsam.Cal3DS2(100, 150, 0, 50, 75, 1, 2, 3, 4)
        wc = to_cal(gc)
        self.assertAlmostEqual(100, wc.fx)
        self.assertAlmostEqual(150, wc.fy)
        self.assertAlmostEqual(0, wc.s)
        self.assertAlmostEqual(50, wc.u0)
        self.assertAlmostEqual(75, wc.v0)
        self.assertAlmostEqual(1, wc.k1)
        self.assertAlmostEqual(2, wc.k2)
        self.assertAlmostEqual(3, wc.p1)
        self.assertAlmostEqual(4, wc.p2)
