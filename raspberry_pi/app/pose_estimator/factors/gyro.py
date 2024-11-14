"""Factory for gyro factors.

The Redux gyro provides integrated position and high-bandwidth
instantaneous velocity, but not average velocity over the report
period, so we just use the position measurement.
"""

# pylint: disable=C0103,E0611,E1101

import gtsam
import numpy as np
from gtsam.noiseModel import Base as SharedNoiseModel  # type:ignore

from app.pose_estimator.numerical_derivative import numericalDerivative11


def h(p0: gtsam.Pose2) -> np.ndarray:
    """Estimated change in yaw, as a 1x1 array."""
    return np.array([p0.rotation().theta()])


def h_H(measured: np.ndarray, p0: gtsam.Pose2, H: list[np.ndarray]) -> np.ndarray:
    """measured: 1x1 array"""
    result = h(p0) - measured
    if H is not None:
        H[0] = numericalDerivative11(h, p0)
    return result


def factorCustom(
    measured: np.ndarray,
    model: SharedNoiseModel,
    p0_key: int,
) -> gtsam.NoiseModelFactor:
    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        p0: gtsam.Pose2 = v.atPose2(this.keys()[0])
        return h_H(measured, p0, H)

    # print("key", gtsam.symbolIndex(p0_key))
    return gtsam.CustomFactor(
        model,
        gtsam.KeyVector([p0_key]),  # type:ignore
        error_func,
    )


def factorNative(
    measured: np.ndarray,
    model: SharedNoiseModel,
    p0_key: int,
) -> gtsam.NoiseModelFactor:
    """See PoseRotationPrior.h"""
    return gtsam.PoseRotationPrior2D(p0_key, gtsam.Pose2(0, 0, measured[0]), model)


def factor(
    measured: np.ndarray,
    model: SharedNoiseModel,
    p0_key: int,
) -> gtsam.NoiseModelFactor:
    return factorNative(measured, model, p0_key)
    # return factorCustom(measured, model, p0_key)
