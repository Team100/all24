# pylint: disable=C0103,E0611,E1101,R0402


import unittest

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore

import app.pose_estimator.factors.apriltag_smooth as apriltag_smooth

KCAL = gtsam.Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, -0.2, 0.1, 0.0, 0.0)
NOISE2 = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))


class AprilTagSmoothTest(unittest.TestCase):
    def test_h_center_1(self) -> None:
        landmark = np.array([1, 0, 0])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3()
        estimate_px: np.ndarray = apriltag_smooth.h_fn(landmark, offset, KCAL)(p0)
        # landmark on the camera bore, so it's at (cx, cy)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        200,
                        200,
                    ]
                ),
                estimate_px,
            )
        )

    def test_h_side_0(self) -> None:
        # higher than the camera
        landmark = np.array([1, 0, 1])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3()
        estimate_px: np.ndarray = apriltag_smooth.h_fn(landmark, offset, KCAL)(p0)
        # landmark above the camera bore, so the 'y' value is less
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        200,
                        20,
                    ]
                ),
                estimate_px,
            )
        )

    def test_h_upper_left(self) -> None:
        # higher than the camera
        landmark = np.array([1, 1, 1])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3()
        estimate_px: np.ndarray = apriltag_smooth.h_fn(landmark, offset, KCAL)(p0)
        # above and to the left, so both x and y are less.
        # (coincidentally right on the edge)
        print("estimate: ", estimate_px)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        0,
                        0,
                    ]
                ),
                estimate_px,
            )
        )

    def test_H_center(self) -> None:
        # as above but with jacobians
        measured = np.array([200, 200])
        landmark = np.array([1, 0, 0])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3()
        H = [np.zeros((3, 2))]
        err_px: np.ndarray = apriltag_smooth.h_H(
            landmark, measured, p0, offset, KCAL, H
        )
        # same case as above
        print("err_px", err_px)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        0,
                        0,
                    ]
                ),
                err_px,
            )
        )
        print("H[0]:\n", H[0])
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0, 200, 200],
                        [0, 0.0, 0],
                    ]
                ),
                H[0],
                atol=0.001,
            )
        )

    def test_H_upper_left(self) -> None:
        # as above but with jacobians
        measured = np.array([0, 0])
        landmark = np.array([1, 1, 1])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3()
        H = [np.zeros((3, 2))]
        err_px: np.ndarray = apriltag_smooth.h_H(
            landmark, measured, p0, offset, KCAL, H
        )
        # same case as above
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        0,
                        0,
                    ]
                ),
                err_px,
            )
        )
        print("H[0]:\n", H[0])
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

    def test_factor(self) -> None:
        # higher than the camera
        measured = np.array([0, 0])
        landmark = np.array([1, 1, 1])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3()
        f: gtsam.NoiseModelFactor = apriltag_smooth.factor(
            landmark,
            measured,
            offset,
            KCAL,
            NOISE2,
            X(0),
        )
        v = gtsam.Values()
        v.insert(X(0), p0)
        err_px = f.unwhitenedError(v)
        # same case as above
        print("err: ", err_px)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        0,
                        0,
                    ]
                ),
                err_px,
            )
        )
