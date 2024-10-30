# pylint: disable=C0103,E0611,E1101,R0402


import unittest

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore

import app.pose_estimator.apriltag as apriltag

KCAL = gtsam.Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, -0.2, 0.1, 0.0, 0.0)
NOISE2 = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))


class AprilTagTest(unittest.TestCase):
    def test_h_center_0(self) -> None:
        landmark = np.array([1, 0, 0])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3()
        calib = gtsam.Cal3DS2()
        result: np.ndarray = apriltag.h_fn(landmark)(p0, offset, calib)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])

    def test_h_center_1(self) -> None:
        landmark = np.array([1, 0, 0])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3()
        result: np.ndarray = apriltag.h_fn(landmark)(p0, offset, KCAL)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(200, result[0])
        self.assertAlmostEqual(200, result[1])

    def test_h_side_0(self) -> None:
        # higher than the camera
        landmark = np.array([1, 0, 1])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3()
        result: np.ndarray = apriltag.h_fn(landmark)(p0, offset, KCAL)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(200, result[0])
        self.assertAlmostEqual(20, result[1])

    def test_h_upper_left(self) -> None:
        # higher than the camera
        landmark = np.array([1, 1, 1])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3()
        result: np.ndarray = apriltag.h_fn(landmark)(p0, offset, KCAL)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])

    def test_H_upper_left(self) -> None:
        # higher than the camera
        measured = np.array([0, 0])
        landmark = np.array([1, 1, 1])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3()
        H = [np.zeros((3, 2)), np.zeros((6, 2)), np.zeros((9, 2))]
        result: np.ndarray = apriltag.h_H(landmark, measured, p0, offset, KCAL, H)
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
                        [-360, 280, 640],
                        [-360, 80, 440],
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
                        [-200, -440, 640, -360, 280, 80],
                        [200, -640, 440, -360, 80, 280],
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
                        [-1, 0, -1, 1, 0, -400, -800, 400, 800],
                        [0, -1, 0, 0, 1, -400, -800, 800, 400],
                    ]
                ),
                H[2],
                atol=0.001,
            )
        )

    def test_factor(self) -> None:
        # higher than the camera
        measured = np.array([0, 0])
        landmark = np.array([1, 1, 1])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3()
        f: gtsam.NoiseModelFactor = apriltag.factor(landmark, measured, NOISE2,
                                                     X(0), X(1), X(2))
        v = gtsam.Values()
        v.insert(X(0), p0)
        v.insert(X(1), offset)
        v.insert(X(2), KCAL)
        result = f.unwhitenedError(v)
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(0, result[0], 2)
        self.assertAlmostEqual(0, result[1], 2)
