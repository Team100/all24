"""Factory for AprilTag detectors.

One type of factor uses constant camera parameters, the other treats
camera parameters as a model variable, for calibration.

This also serves as a template for parameterized estimators -- in this
case, the parameters are the landmark location, camera calibration,
and camera offset.
"""

# pylint: disable=C0103,E0611,E1101,R0913
from typing import Callable

import gtsam
import numpy as np
from gtsam.noiseModel import Base as SharedNoiseModel  # type:ignore

from app.pose_estimator.numerical_derivative import numericalDerivative11


def h_fn(
    landmark: np.ndarray,
    offset: gtsam.Pose3,
    calib: gtsam.Cal3DS2,
) -> Callable[[gtsam.Pose2], np.ndarray]:
    """Returns a pixel estimation function for constant landmark,
    constant offset, and constant calibration, with variable pose.
    The landmark is field position of a tag corner."""

    def h(p0: gtsam.Pose2) -> np.ndarray:
        """Estimated pixel location of the target."""
        camera_pose = gtsam.Pose3(p0).compose(offset)
        camera = gtsam.PinholeCameraCal3DS2(camera_pose, calib)
        # Camera.project() will throw CheiralityException sometimes
        # so be sure to use GTSAM_THROW_CHEIRALITY_EXCEPTION=OFF
        # (as nightly.yml does).
        return camera.project(landmark)

    return h


def h_H(
    landmark: np.ndarray,
    measured: np.ndarray,
    p0: gtsam.Pose2,
    offset: gtsam.Pose3,
    calib: gtsam.Cal3DS2,
    H: list[np.ndarray],
) -> np.ndarray:
    """Error function (in pixels), including Jacobians, H."""
    h = h_fn(landmark, offset, calib)
    estimate = h(p0)
    # print("ESTIMATE", estimate)
    # print("MEASURED", measured)
    result = estimate - measured
    if H is not None:
        H[0] = numericalDerivative11(h, p0)
    return result


def factorCustom(
    landmark: np.ndarray,
    measured: np.ndarray,
    offset: gtsam.Pose3,
    calib: gtsam.Cal3DS2,
    model: SharedNoiseModel,
    p0_key: int,
) -> gtsam.NoiseModelFactor:
    """using constant offset and calibration, use this for smoothing.

    landmark: field coordinates
    measured: pixel coordinate"""

    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        p0: gtsam.Pose2 = v.atPose2(this.keys()[0])
        return h_H(landmark, measured, p0, offset, calib, H)

    return gtsam.CustomFactor(
        model,
        gtsam.KeyVector([p0_key]),  # type:ignore
        error_func,
    )


def factorNative(
    landmark: np.ndarray,
    measured: np.ndarray,
    offset: gtsam.Pose3,
    calib: gtsam.Cal3DS2,
    model: SharedNoiseModel,
    p0_key: int,
) -> gtsam.NoiseModelFactor:
    return gtsam.PlanarProjectionFactor1(
        p0_key, landmark, measured, offset, calib, model
    )


def factor(
    landmark: np.ndarray,
    measured: np.ndarray,
    offset: gtsam.Pose3,
    calib: gtsam.Cal3DS2,
    model: SharedNoiseModel,
    p0_key: int,
) -> gtsam.NoiseModelFactor:
    return factorNative(landmark, measured, offset, calib, model, p0_key)
    # return factorCustom(landmark, measured, offset, calib, model, p0_key)
