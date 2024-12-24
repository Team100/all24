# pylint: disable=C0103,E0611,E1101,R0402


import unittest

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import C, K, X  # type:ignore

import app.pose_estimator.factors.apriltag_calibrate as apriltag_calibrate

# example calibration, focal length 200, pixel center 200, a little bit
# of distortion.
NOISE2 = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))


class AprilTagCalibrateTest(unittest.TestCase):
    def test_h_center_1(self) -> None:
        landmark = np.array([1, 0, 0])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3(
            gtsam.Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
            gtsam.Point3(0, 0, 0),  # type: ignore
        )
        KCAL = gtsam.Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, -0.2, 0.1)
        estimate_px: np.ndarray = apriltag_calibrate.h_fn(landmark)(p0, offset, KCAL)
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
        offset = gtsam.Pose3(
            gtsam.Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
            gtsam.Point3(0, 0, 0),  # type: ignore
        )
        KCAL = gtsam.Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, -0.2, 0.1)
        estimate_px: np.ndarray = apriltag_calibrate.h_fn(landmark)(p0, offset, KCAL)
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
        offset = gtsam.Pose3(
            gtsam.Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
            gtsam.Point3(0, 0, 0),  # type: ignore
        )
        KCAL = gtsam.Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, -0.2, 0.1)
        estimate_px: np.ndarray = apriltag_calibrate.h_fn(landmark)(p0, offset, KCAL)
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

    def test_H_upper_left(self) -> None:
        # as above but with jacobians
        measured = np.array([0, 0])
        landmark = np.array([1, 1, 1])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3(
            gtsam.Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
            gtsam.Point3(0, 0, 0),  # type: ignore
        )
        KCAL = gtsam.Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, -0.2, 0.1)
        H = [np.zeros((3, 2)), np.zeros((6, 2)), np.zeros((9, 2))]
        err_px: np.ndarray = apriltag_calibrate.h_H(
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
        # remember camera is z-fwd now
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [440, -640, -200, -280, -80, -360],
                        [640, -440, 200, -80, -280, -360],
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

    def test_H_upper_left2(self) -> None:
        # as above but with jacobians
        measured = np.array([0, 0])
        landmark = np.array([1, 1, 1])
        p0 = gtsam.Pose2()
        offset = gtsam.Pose3(
            gtsam.Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
            gtsam.Point3(0, 0, 0),  # type: ignore
        )
        KCAL = gtsam.Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, 0, 0)
        H = [np.zeros((3, 2)), np.zeros((6, 2)), np.zeros((9, 2))]
        err_px: np.ndarray = apriltag_calibrate.h_H(
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
        print("H[1]:\n", H[1])
        print("H[2]:\n", H[2])
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [-200, 200, 400],
                        [-200, 0, 200],
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
                        [200, -400, -200, -200, 0, -200],
                        [400, -200, 200, 0, -200, -200],
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

    def test_H_upper_left3(self) -> None:
        # offset rotates the center near the landmark
        measured = np.array([238, 278])
        landmark = np.array([1, 1, 1])
        p0 = gtsam.Pose2()
        # Remember Ypr is intrinsic
        offset = gtsam.Pose3(gtsam.Rot3().Ypr(1, -1, 0), np.array([0, 0, 0])).compose(
            gtsam.Pose3(
                gtsam.Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
                gtsam.Point3(0, 0, 0),
            )
        )  # type: ignore
        print("OFFSET", offset)
        KCAL = gtsam.Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, 0, 0)
        H = [np.zeros((3, 2)), np.zeros((6, 2)), np.zeros((9, 2))]
        err_px: np.ndarray = apriltag_calibrate.h_H(
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
                atol=1,
            )
        )
        print("H[0]:\n", H[0])
        print("H[1]:\n", H[1])
        print("H[2]:\n", H[2])
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [-99, 79, 178],
                        [-43, -67, -24],
                    ]
                ),
                H[0],
                atol=1,
            )
        )
        # camera is z-fwd
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [14, -207, 78, -126, 0, 24],
                        [231, -14, -38, 0, -126, 49],
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
                        [0.190, 0, 0.392, 1, 0, 7.191, 1.363, 29.732, 52.303],
                        [0, 0.392, 0, 0, 1, 14.861, 2.817, 99.364, 29.732],
                    ]
                ),
                H[2],
                atol=0.001,
            )
        )

    def test_factor(self) -> None:
        # same as above
        measured = np.array([0, 0])
        landmark = np.array([1, 1, 1])
        p0 = gtsam.Pose2()
        # camera is now z-forward y-down
        offset = gtsam.Pose3(
            gtsam.Rot3(0, 0, 1, -1, 0, 0, 0, -1, 0), np.array([0, 0, 0])
        )
        KCAL = gtsam.Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, -0.2, 0.1)
        f: gtsam.NoiseModelFactor = apriltag_calibrate.factor(
            landmark,
            measured,
            NOISE2,
            X(0),
            C(0),
            K(0),
        )
        v = gtsam.Values()
        v.insert(X(0), p0)
        v.insert(C(0), offset)
        v.insert(K(0), KCAL)
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
