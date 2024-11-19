# pylint: disable=C0103,E0611,E1101,R0402

import math
import unittest

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore
from wpimath.geometry import Pose2d, Twist2d

import app.pose_estimator.factors.odometry as odometry

NOISE3 = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))


class OdometryTest(unittest.TestCase):
    def test_odometry_factor(self) -> None:
        """No movement in either estimate or measurement, so no error."""
        # The CustomFactor wrapper makes this hard to test more thoroughly.
        t = Twist2d(0, 0, 0)
        f: gtsam.NoiseModelFactor = odometry.factor(t, NOISE3, X(0), X(1))
        # print("keys", f.keys())
        v = gtsam.Values()
        p0 = gtsam.Pose2()
        p1 = gtsam.Pose2()
        v.insert(X(0), p0)
        v.insert(X(1), p1)
        result = f.unwhitenedError(v)
        self.assertEqual(3, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])
        self.assertAlmostEqual(0, result[2])

    def test_odo_H(self) -> None:
        """No movement, also include Jacobians."""
        # use the inner odo_H factor without the CustomFactor wrapper.
        t = Twist2d(0, 0, 0)
        measured = np.array([t.dx, t.dy, t.dtheta])
        p0 = gtsam.Pose2()
        p1 = gtsam.Pose2()
        H = [np.zeros((3, 3)), np.zeros((3, 3))]
        result = odometry.h_H(measured, p0, p1, H)
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

    def test_h(self) -> None:
        """Verify h alone"""
        # (1,1) facing +y
        p0 = gtsam.Pose2()
        p1 = gtsam.Pose2(1, 1, math.pi / 2)
        result: np.ndarray = odometry.h(p0, p1)
        self.assertEqual(3, len(result))
        self.assertAlmostEqual(math.pi / 2, result[0])
        self.assertAlmostEqual(0, result[1])
        self.assertAlmostEqual(math.pi / 2, result[2])

    def test_odo_H2(self) -> None:
        """Estimate and measurement are the same, drive ahead while turning left."""
        t = Twist2d(math.pi / 2, 0, math.pi / 2)
        testP0 = Pose2d()
        testP1 = testP0.exp(t)
        # we should end up at (1,1) facing +y.
        self.assertAlmostEqual(1, testP1.X())
        self.assertAlmostEqual(1, testP1.Y())
        self.assertAlmostEqual(math.pi / 2, testP1.rotation().radians())
        measured = np.array([t.dx, t.dy, t.dtheta])
        p0 = gtsam.Pose2()
        # here's (1,1) facing +y again
        p1 = gtsam.Pose2(1, 1, math.pi / 2)
        H = [np.zeros((3, 3)), np.zeros((3, 3))]
        result = odometry.h_H(measured, p0, p1, H)
        self.assertEqual(3, len(result))
        # no error
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
        """Same left turn, but the estimate is behind a little."""
        # drive ahead while turning left
        t = Twist2d(math.pi / 2, 0, math.pi / 2)
        testP0 = Pose2d()
        testP1 = testP0.exp(t)
        # we should end up here
        self.assertAlmostEqual(1, testP1.X())
        self.assertAlmostEqual(1, testP1.Y())
        self.assertAlmostEqual(math.pi / 2, testP1.rotation().radians())
        measured = np.array([t.dx, t.dy, t.dtheta])
        p0 = gtsam.Pose2()
        # now there's an error which should map to a pure x error in the twist
        # this position is due to just not driving far enough
        p1 = gtsam.Pose2(0.9, 0.9, math.pi / 2)
        H = [np.zeros((3, 3)), np.zeros((3, 3))]
        result = odometry.h_H(measured, p0, p1, H)
        self.assertEqual(3, len(result))
        # the tangent-space error is purely in x
        self.assertAlmostEqual(-0.157, result[0], 2)
        self.assertAlmostEqual(0, result[1], 2)
        self.assertAlmostEqual(0, result[2])
        print("H[0]:\n", H[0])
        print("H[1]:\n", H[1])
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [-0.785, -0.785, -0.193],
                        [0.785, -0.785, -0.706],
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
                        [0.785, -0.785, 0.193],
                        [0.785, 0.785, -0.706],
                        [0, 0, 1],
                    ]
                ),
                H[1],
                atol=0.001,
            )
        )

    def test_odo_H4(self) -> None:
        """move diagonally without rotating"""
        t = Twist2d(1, 1, 0)
        testP0 = Pose2d()
        testP1 = testP0.exp(t)
        # we should end up here
        self.assertAlmostEqual(1, testP1.X())
        self.assertAlmostEqual(1, testP1.Y())
        self.assertAlmostEqual(0, testP1.rotation().radians())
        measured = np.array([t.dx, t.dy, t.dtheta])
        p0 = gtsam.Pose2()
        # now there's an error which should be symmetrical
        p1 = gtsam.Pose2(0.9, 0.9, 0)
        H = [np.zeros((3, 3)), np.zeros((3, 3))]
        result = odometry.h_H(measured, p0, p1, H)
        self.assertEqual(3, len(result))
        self.assertAlmostEqual(-0.1, result[0], 2)
        self.assertAlmostEqual(-0.1, result[1], 2)
        self.assertAlmostEqual(0, result[2])
        print("H[0]:\n", H[0])
        print("H[1]:\n", H[1])
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [-1, 0, 0.45],
                        [0, -1, -0.45],
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
                        [1, 0, 0.45],
                        [0, 1, -0.45],
                        [0, 0, 1],
                    ]
                ),
                H[1],
                atol=0.001,
            )
        )
