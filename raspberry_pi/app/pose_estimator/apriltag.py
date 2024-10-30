"""Factory for AprilTag detectors.

One type of factor uses constant camera parameters, the other treats
camera parameters as a model variable, for calibration.

This also serves as a template for parameterized estimators -- in this
case, the parameter is the landmark location.
"""

# pylint: disable=C0103,E0611,E1101,R0913
from typing import Callable

import gtsam
import numpy as np
from gtsam.noiseModel import Base as SharedNoiseModel

from app.pose_estimator.numerical_derivative import (
    numericalDerivative31,
    numericalDerivative32,
    numericalDerivative33,
)

# camera "zero" is facing +z; this turns it to face +x
CAM_COORD = gtsam.Pose3(
    gtsam.Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])), gtsam.Point3(0, 0, 0)
)


def h_fn(
    landmark: np.ndarray,
) -> Callable[[gtsam.Pose2, gtsam.Pose3, gtsam.Cal3DS2], np.ndarray]:
    """landmark is field position of a tag corner."""

    def h(p0: gtsam.Pose2, offset: gtsam.Pose3, calib: gtsam.Cal3DS2) -> np.ndarray:
        """estimated pixel location of the target"""
        # this is x-forward z-up
        offset_pose = gtsam.Pose3(p0).compose(offset)
        # this is z-forward y-down
        camera_pose = offset_pose.compose(CAM_COORD)
        camera = gtsam.PinholeCameraCal3DS2(camera_pose, calib)
        return camera.project(landmark)

    return h


def h_H(
    landmark: np.ndarray,
    measured: np.array,
    p0: gtsam.Pose2,
    offset: gtsam.Pose3,
    calib: gtsam.Cal3DS2,
    H: list[np.ndarray],
) -> np.ndarray:
    h = h_fn(landmark)
    result = h(p0, offset, calib) - measured
    if H is not None:
        H[0] = numericalDerivative31(h, p0, offset, calib)
        H[1] = numericalDerivative32(h, p0, offset, calib)
        H[2] = numericalDerivative33(h, p0, offset, calib)
    return result


def factor(
    landmark: np.ndarray,
    measured: np.ndarray,
    model: SharedNoiseModel,
    p0_key: gtsam.Symbol,
    offset_key: gtsam.Symbol,
    calib_key: gtsam.Symbol,
) -> gtsam.NonlinearFactor:
    """landmark: field coordinates
    measured: pixel coordinate"""

    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        p0: gtsam.Pose2 = v.atPose2(this.keys()[0])
        offset: gtsam.Pose3 = v.atPose3(this.keys()[1])
        calib: gtsam.Cal3DS2 = v.atCal3DS2(this.keys()[2])

        return h_H(landmark, measured, p0, offset, calib, H)

    return gtsam.CustomFactor(
        model,
        gtsam.KeyVector([p0_key, offset_key, calib_key]),
        error_func,
    )
