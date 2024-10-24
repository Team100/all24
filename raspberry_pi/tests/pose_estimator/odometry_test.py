# pylint: disable=C0103,E0611,E1101

import math
import unittest

import gtsam
import numpy as np
from gtsam import (
    BetweenFactorDouble,
    BetweenFactorPose2,
    CustomFactor,
    FixedLagSmoother,
    ISAM2GaussNewtonParams,
    KeyVector,
    NonlinearFactor,
    NonlinearFactorGraph,
    Point2,
    Point3,
    Pose2,
    Pose3,
    Rot2,
    Rot3,
    Values,
)
from gtsam.noiseModel import Base as SharedNoiseModel
from gtsam.noiseModel import Diagonal
from gtsam.symbol_shorthand import X  # robot pose
from wpimath.geometry import Twist2d, Pose2d

from app.pose_estimator.odometry import odometry_factor, odo_H

NOISE3 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))


class OdometryTest(unittest.TestCase):
    def test_odo_factor(self) -> None:
        t = Twist2d(0, 0, 0)
        f = odometry_factor(t, NOISE3, 0, 1)
        print("keys", f.keys())
        v = gtsam.Values()
        p0 = Pose2()
        p1 = Pose2()
        v.insert(0, p0)
        v.insert(1, p1)
        result = f.unwhitenedError(v)
        self.assertEqual(3, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])
        self.assertAlmostEqual(0, result[2])

    def test_odo_H(self) -> None:
        t = Twist2d(0, 0, 0)
        measured = [t.dx, t.dy, t.dtheta]
        p0 = Pose2()
        p1 = Pose2()
        H = [np.zeros((3, 3)), np.zeros((3, 3))]
        result = odo_H(measured, p0, p1, H)
        self.assertEqual(3, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])
        self.assertAlmostEqual(0, result[2])
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [-1, 0, 0],
                        [0, -1, 0],
                        [0, 0, -1],
                    ]
                ),
                H[0],
            )
        )
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1],
                    ]
                ),
                H[1],
            )
        )

    def test_odo_H2(self) -> None:
        # drive ahead while turning left
        t = Twist2d(math.pi / 2, 0, math.pi / 2)
        testP0 = Pose2d()
        testP1 = testP0.exp(t)
        # we should end up here
        self.assertAlmostEqual(1, testP1.X())
        self.assertAlmostEqual(1, testP1.Y())
        self.assertAlmostEqual(math.pi / 2, testP1.rotation().radians())
        measured = [t.dx, t.dy, t.dtheta]
        p0 = Pose2()
        p1 = Pose2(1, 1, math.pi / 2)
        H = [np.zeros((3, 3)), np.zeros((3, 3))]
        result = odo_H(measured, p0, p1, H)
        self.assertEqual(3, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])
        self.assertAlmostEqual(0, result[2])
        print("H[0]:\n", H[0])
        print("H[1]:\n", H[1])
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [-0.785, -0.785, -0.214],
                        [0.785, -0.785, -0.785],
                        [0, 0, -1],
                    ]
                ),
                H[0],
                atol=0.001,
            )
        )
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.785, -0.785, 0.214],
                        [0.785, 0.785, -0.785],
                        [0, 0, 1],
                    ]
                ),
                H[1],
                atol=0.001,
            )
        )


    def test_odo_H3(self) -> None:
        # drive ahead while turning left
        t = Twist2d(math.pi / 2, 0, math.pi / 2)
        testP0 = Pose2d()
        testP1 = testP0.exp(t)
        # we should end up here
        self.assertAlmostEqual(1, testP1.X())
        self.assertAlmostEqual(1, testP1.Y())
        self.assertAlmostEqual(math.pi / 2, testP1.rotation().radians())
        measured = [t.dx, t.dy, t.dtheta]
        p0 = Pose2()
        # now there's an error
        p1 = Pose2(1, 0.9, math.pi / 2)
        H = [np.zeros((3, 3)), np.zeros((3, 3))]
        result = odo_H(measured, p0, p1, H)
        self.assertEqual(3, len(result))
        self.assertAlmostEqual(-0.078, result[0], 2)
        self.assertAlmostEqual(-0.078, result[1], 2)
        self.assertAlmostEqual(0, result[2])
        print("H[0]:\n", H[0])
        print("H[1]:\n", H[1])
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [-0.785, -0.785, -0.214],
                        [0.785, -0.785, -0.785],
                        [0, 0, -1],
                    ]
                ),
                H[0],
                atol=0.001,
            )
        )
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.785, -0.785, 0.214],
                        [0.785, 0.785, -0.785],
                        [0, 0, 1],
                    ]
                ),
                H[1],
                atol=0.001,
            )
        )

    def test_odo_H4(self) -> None:
        # move diagonally without rotating
        t = Twist2d(1, 1, 0)
        testP0 = Pose2d()
        testP1 = testP0.exp(t)
        # we should end up here
        self.assertAlmostEqual(1, testP1.X())
        self.assertAlmostEqual(1, testP1.Y())
        self.assertAlmostEqual(0, testP1.rotation().radians())
        measured = [t.dx, t.dy, t.dtheta]
        p0 = Pose2()
        # now there's an error
        p1 = Pose2(1, 0.9, 0)
        H = [np.zeros((3, 3)), np.zeros((3, 3))]
        result = odo_H(measured, p0, p1, H)
        self.assertEqual(3, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(-0.1, result[1], 2)
        self.assertAlmostEqual(0, result[2])
        print("H[0]:\n", H[0])
        print("H[1]:\n", H[1])
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [-0.785, -0.785, -0.214],
                        [0.785, -0.785, -0.785],
                        [0, 0, -1],
                    ]
                ),
                H[0],
                atol=0.001,
            )
        )
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.785, -0.785, 0.214],
                        [0.785, 0.785, -0.785],
                        [0, 0, 1],
                    ]
                ),
                H[1],
                atol=0.001,
            )
        )