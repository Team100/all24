"""AprilTag factor for smoothing in batch.

A batch means the set of AprilTag corners seen in a single frame.
"""

# pylint: disable=C0103,E0611,E1101,R0913
from typing import Callable

import gtsam
import numpy as np
from gtsam.noiseModel import Base as SharedNoiseModel  # type:ignore

from app.pose_estimator.numerical_derivative import numericalDerivative11



def h_fn(
    landmarks: list[np.ndarray],
    offset: gtsam.Pose3,
    calib: gtsam.Cal3DS2,
) -> Callable[[gtsam.Pose2], np.ndarray]:
    """Returns a pixel estimation function for an array of landmarks.
    The estimate is a single flat array with all the points, concatenated,
    i.e. (x0, y0, x1, y1, x2, y2, ...)"""

    def h(p0: gtsam.Pose2) -> np.ndarray:
        """estimated pixel location of the target"""
        camera_pose = gtsam.Pose3(p0).compose(offset)
        camera = gtsam.PinholeCameraCal3DS2(camera_pose, calib)
        # Camera.project() will throw CheiralityException sometimes
        # so be sure to use GTSAM_THROW_CHEIRALITY_EXCEPTION=OFF
        # (as nightly.yml does).
        return np.concatenate([camera.project(x) for x in landmarks])

    return h


def h_H(
    landmarks: list[np.ndarray],
    measured: np.ndarray,
    p0: gtsam.Pose2,
    offset: gtsam.Pose3,
    calib: gtsam.Cal3DS2,
    H: list[np.ndarray],
) -> np.ndarray:
    """measured: concatenation of px"""
    h = h_fn(landmarks, offset, calib)
    result = h(p0) - measured
    if H is not None:
        H[0] = numericalDerivative11(h, p0)

    return result


def factor(
    landmarks: list[np.ndarray],
    measured: np.ndarray,
    offset: gtsam.Pose3,
    calib: gtsam.Cal3DS2,
    model: SharedNoiseModel,
    p0_key: int,
) -> gtsam.NoiseModelFactor:
    """using constant offset and calibration, use this for smoothing.

    landmark: list of field coordinates
    measured: concatenation of pixel coordinates"""

    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        p0: gtsam.Pose2 = v.atPose2(this.keys()[0])
        return h_H(landmarks, measured, p0, offset, calib, H)

    return gtsam.CustomFactor(
        model,
        gtsam.KeyVector([p0_key]),  # type:ignore
        error_func,
    )
