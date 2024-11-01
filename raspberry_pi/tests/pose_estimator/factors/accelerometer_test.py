# pylint: disable=C0103,E0611,E1101,R0402

import unittest

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore

import app.pose_estimator.factors.accelerometer as accelerometer

NOISE3 = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))


class AccelerometerTest(unittest.TestCase):
    def test_h_0(self) -> None:
        """motionless"""
        p0 = gtsam.Pose2()
        p1 = gtsam.Pose2()
        p2 = gtsam.Pose2()
        dt1 = 0.02
        dt2 = 0.02
        result: np.ndarray = accelerometer.h(p0, p1, p2, dt1, dt2)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])

    def test_h_1(self) -> None:
        """constant linear velocity"""
        p0 = gtsam.Pose2(0, 0, 0)
        p1 = gtsam.Pose2(1, 0, 0)
        p2 = gtsam.Pose2(2, 0, 0)
        dt1 = 0.02
        dt2 = 0.02
        result: np.ndarray = accelerometer.h(p0, p1, p2, dt1, dt2)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])

    def test_h_1a(self) -> None:
        """constant angular velocity"""
        p0 = gtsam.Pose2(0, 0, 0)
        p1 = gtsam.Pose2(0, 0, 1)
        p2 = gtsam.Pose2(0, 0, 2)
        dt1 = 0.02
        dt2 = 0.02
        result: np.ndarray = accelerometer.h(p0, p1, p2, dt1, dt2)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])

    def test_h_2(self) -> None:
        """linear acceleration"""
        p0 = gtsam.Pose2(0, 0, 0)
        p1 = gtsam.Pose2(1, 0, 0)
        p2 = gtsam.Pose2(4, 0, 0)
        dt1 = 0.02
        dt2 = 0.02
        result: np.ndarray = accelerometer.h(p0, p1, p2, dt1, dt2)
        self.assertEqual(2, len(result))
        # v1 is 50
        # v2 is 150
        # delta v is 100, delta t is 0.02, so a is 5000
        self.assertAlmostEqual(5000, result[0])
        self.assertAlmostEqual(0, result[1])

    def test_h_2a(self) -> None:
        """angular acceleration is not measured"""
        p0 = gtsam.Pose2(0, 0, 0)
        p1 = gtsam.Pose2(0, 0, 1)
        p2 = gtsam.Pose2(0, 0, 4)
        dt1 = 0.02
        dt2 = 0.02
        result: np.ndarray = accelerometer.h(p0, p1, p2, dt1, dt2)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])

    def test_h_coriolis(self) -> None:
        """constant linear and angular speed: big coriolis effect"""
        p0 = gtsam.Pose2(0, 0, 0)
        p1 = gtsam.Pose2(1, 0, 1)
        p2 = gtsam.Pose2(2, 0, 2)
        dt1 = 0.02
        dt2 = 0.02
        result: np.ndarray = accelerometer.h(p0, p1, p2, dt1, dt2)
        self.assertEqual(2, len(result))
        # turning to the left, so we need -y and -x to keep going the same direction.
        self.assertAlmostEqual(-7305, result[0], 0)
        self.assertAlmostEqual(-1720, result[1], 0)

    def test_h_coriolis_realistic(self) -> None:
        """Coriolis effect with realistic numbers"""
        p0 = gtsam.Pose2(0, 0, 0)
        p1 = gtsam.Pose2(0.02, 0, 0.02)
        p2 = gtsam.Pose2(0.04, 0, 0.04)
        dt1 = 0.02
        dt2 = 0.02
        result: np.ndarray = accelerometer.h(p0, p1, p2, dt1, dt2)
        self.assertEqual(2, len(result))
        # we've only turned a little, so only slowed a little
        self.assertAlmostEqual(-0.08, result[0], 2)
        # but it's still a sizeable acceleration in y
        # is this right?
        # translation velocity is 1
        # y component goes from -0.01 to -0.03 which is
        # delta-v of just about 0.02, in time 0.02, for a velocity
        # of about -1.  to maintain the correct path, the inertial force is
        # also about -1, and the reference frame is carried (with respect to itself)
        # with that same velocity, so the true velocity (i.e. the coriolis term)
        # is about -2, and the net is -3.
        self.assertAlmostEqual(-3, result[1], 2)

    def test_H(self) -> None:
        p0 = gtsam.Pose2()
        p1 = gtsam.Pose2()
        p2 = gtsam.Pose2()
        dt1 = 0.02
        dt2 = 0.02
        measured = np.array([0, 0])
        H = [np.zeros((2, 3)), np.zeros((2, 3)), np.zeros((2, 3))]
        result: np.ndarray = accelerometer.h_H(measured, p0, p1, p2, dt1, dt2, H)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])
        print("H[0]:\n", H[0])
        print("H[1]:\n", H[1])
        print("H[2]:\n", H[2])
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [2500, 0, 0],
                        [0, 2500, 0],
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
                        [-5000, 0, 0],
                        [0, -5000, 0],
                    ]
                ),
                H[1],
                atol=0.001,
            )
        )
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [2500, 0, 0],
                        [0, 2500, 0],
                    ]
                ),
                H[2],
                atol=0.001,
            )
        )

    def test_H_accel(self) -> None:
        p0 = gtsam.Pose2(0, 0, 0)
        p1 = gtsam.Pose2(1, 0, 0)
        p2 = gtsam.Pose2(4, 0, 0)
        dt1 = 0.02
        dt2 = 0.02
        measured = np.array([0, 0])
        H = [np.zeros((2, 3)), np.zeros((2, 3)), np.zeros((2, 3))]
        result: np.ndarray = accelerometer.h_H(measured, p0, p1, p2, dt1, dt2, H)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(5000, result[0])
        self.assertAlmostEqual(0, result[1])
        print("H[0]:\n", H[0])
        print("H[1]:\n", H[1])
        print("H[2]:\n", H[2])
        # i dunno if these jacobians are right.
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [2500, 0, 0],
                        [0, 2500, 1250],
                    ]
                ),
                H[0],
                atol=1,
            )
        )
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [-5000, 0, 0],
                        [0, -5000, 12500],
                    ]
                ),
                H[1],
                atol=1,
            )
        )
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [2500, 0, 0],
                        [0, 2500, -18750],
                    ]
                ),
                H[2],
                atol=1,
            )
        )

    def test_H_omega(self) -> None:
        """Constant omega, zero linear velocity and accel.
        I think the Jacobians here mostly describe the coriolis effect."""
        p0 = gtsam.Pose2(0, 0, 0)
        p1 = gtsam.Pose2(0, 0, 1)
        p2 = gtsam.Pose2(0, 0, 2)
        dt1 = 0.02
        dt2 = 0.02
        measured = np.array([0, 0])
        H = [np.zeros((2, 3)), np.zeros((2, 3)), np.zeros((2, 3))]
        result: np.ndarray = accelerometer.h_H(measured, p0, p1, p2, dt1, dt2, H)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])
        print("H[0]:\n", H[0])
        print("H[1]:\n", H[1])
        print("H[2]:\n", H[2])
        # i dunno if these jacobians are right.
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [2288, 1250, 0],
                        [-1250, 2288, 0],
                    ]
                ),
                H[0],
                atol=1,
            )
        )
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [-2076, -4576, 0],
                        [4576, -2076, 0],
                    ]
                ),
                H[1],
                atol=1,
            )
        )
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [4788, 3326, 0],
                        [-3326, 4788, 0],
                    ]
                ),
                H[2],
                atol=1,
            )
        )

    def test_H_alpha(self) -> None:
        """The accelerometer doesn't provide alpha so the measurement is always zero there."""
        p0 = gtsam.Pose2(0, 0, 0)
        p1 = gtsam.Pose2(0, 0, 1)
        p2 = gtsam.Pose2(0, 0, 4)
        dt1 = 0.02
        dt2 = 0.02
        measured = np.array([0, 0])
        H = [np.zeros((2, 3)), np.zeros((2, 3)), np.zeros((2, 3))]
        result: np.ndarray = accelerometer.h_H(measured, p0, p1, p2, dt1, dt2, H)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])
        print("H[0]:\n", H[0])
        print("H[1]:\n", H[1])
        print("H[2]:\n", H[2])
        # i dunno if these jacobians are right.
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [2288, 1250, 0],
                        [-1250, 2288, 0],
                    ]
                ),
                H[0],
                atol=1,
            )
        )
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [19945, -4095, 0],
                        [4095, 19945, 0],
                    ]
                ),
                H[1],
                atol=1,
            )
        )
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [22765, -2154, 0],
                        [2154, 22765, 0],
                    ]
                ),
                H[2],
                atol=1,
            )
        )

    def test_accel_factor(self) -> None:
        x_accel = 0
        y_accel = 0
        dt = 0.02

        f: gtsam.NoiseModelFactor = accelerometer.factor(
            x_accel, y_accel, dt, dt, NOISE3, X(0), X(1), X(2)
        )
        v = gtsam.Values()
        p0 = gtsam.Pose2()
        p1 = gtsam.Pose2()
        p2 = gtsam.Pose2()
        v.insert(X(0), p0)
        v.insert(X(1), p1)
        v.insert(X(2), p2)
        result = f.unwhitenedError(v)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])

    def test_accel_factor2(self) -> None:
        x_accel = 0
        y_accel = 0
        dt = 0.02

        f: gtsam.NonlinearFactor = accelerometer.factor(
            x_accel, y_accel, dt, dt, NOISE3, X(0), X(1), X(2)
        )
        v = gtsam.Values()
        p0 = gtsam.Pose2(0, 0, 0)
        p1 = gtsam.Pose2(0.01, 0, 0)
        p2 = gtsam.Pose2(0.04, 0, 0)
        v.insert(X(0), p0)
        v.insert(X(1), p1)
        v.insert(X(2), p2)
        result = f.unwhitenedError(v)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(50, result[0])
        self.assertAlmostEqual(0, result[1])

    def test_accel_factor3(self) -> None:
        x_accel = 0
        y_accel = 0
        dt = 0.02

        f: gtsam.NonlinearFactor = accelerometer.factor(
            x_accel, y_accel, dt, dt, NOISE3, X(0), X(1), X(2)
        )
        v = gtsam.Values()
        p0 = gtsam.Pose2(0, 0, 0)
        p1 = gtsam.Pose2(0.02, 0, 0.02)
        p2 = gtsam.Pose2(0.04, 0, 0.04)
        v.insert(X(0), p0)
        v.insert(X(1), p1)
        v.insert(X(2), p2)
        result = f.unwhitenedError(v)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(-0.08, result[0], 2)
        self.assertAlmostEqual(-3, result[1], 2)
