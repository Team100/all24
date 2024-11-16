# pylint: disable=C0103,E0611,E1101,R0402

import unittest

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore

import app.pose_estimator.factors.gyro as gyro

NOISE1 = noiseModel.Diagonal.Sigmas(np.array([0.1]))


class GyroTest(unittest.TestCase):
    def test_h_0(self) -> None:
        """motionless"""
        p0 = gtsam.Pose2()
        result: np.ndarray = gyro.h(p0)
        self.assertEqual(1, len(result))
        self.assertAlmostEqual(0, result[0])

    def test_h_1(self) -> None:
        """theta = 1"""
        p0 = gtsam.Pose2(0, 0, 1)
        result: np.ndarray = gyro.h(p0)
        self.assertEqual(1, len(result))
        self.assertAlmostEqual(1, result[0])

    def test_H_0(self) -> None:
        p0 = gtsam.Pose2(0, 0, 0)
        measured = np.array([0])
        H = [np.zeros((3, 3))]
        result: np.ndarray = gyro.h_H(measured, p0, H)
        self.assertEqual(1, len(result))
        self.assertAlmostEqual(0, result[0])
        print("H[0]:\n", H[0])
        self.assertTrue(
            np.allclose(
                np.array([[0, 0, 1]]),
                H[0],
                atol=0.001,
            )
        )

    def test_H_1(self) -> None:
        # estimate is 1, measurement is 0, so error is 1
        p0 = gtsam.Pose2(0, 0, 1)
        measured = np.array([0])
        H = [np.zeros((3, 3))]
        result: np.ndarray = gyro.h_H(measured, p0, H)
        self.assertEqual(1, len(result))
        self.assertAlmostEqual(1, result[0])
        print("H[0]:\n", H[0])
        self.assertTrue(
            np.allclose(
                np.array([[0, 0, 1]]),
                H[0],
                atol=0.001,
            )
        )

    def test_factor_0(self) -> None:
        measured = np.array([0])
        f: gtsam.NoiseModelFactor = gyro.factor(measured, NOISE1, X(0))
        v = gtsam.Values()
        p0 = gtsam.Pose2(0, 0, 0)
        v.insert(X(0), p0)
        result = f.unwhitenedError(v)
        self.assertEqual(1, len(result))
        self.assertAlmostEqual(0, result[0])

    def test_factor_1(self) -> None:
        """both estimate and measurement are 1"""
        measured = np.array([1])
        f: gtsam.NonlinearFactor = gyro.factor(measured, NOISE1, X(0))
        v = gtsam.Values()
        p0 = gtsam.Pose2(0, 0, 1)
        v.insert(X(0), p0)
        result = f.unwhitenedError(v)
        self.assertEqual(1, len(result))
        self.assertAlmostEqual(0, result[0])
