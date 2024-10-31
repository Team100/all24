"""Factory for gyro factors.

The Redux gyro provides integrated position and high-bandwidth
instantaneous velocity, but not average velocity over the report
period, so we use position measurement and calculate deltas,
just like we do for odometry.  You can think of the gyro as a
rotation-only odometry factor.
"""

# pylint: disable=C0103,E0611,E1101

import gtsam
import numpy as np
from gtsam.noiseModel import Base as SharedNoiseModel  # type:ignore

from app.pose_estimator.numerical_derivative import (
    numericalDerivative21,
    numericalDerivative22,
)


def h(p0: gtsam.Pose2, p1: gtsam.Pose2) -> np.ndarray:
    """Estimated change in yaw, as a 1x1 array."""
    # logmap in this case is really just "minus"
    # since the rotation manifold is the tangent space.
    return p0.rotation().logmap(p1.rotation())


def h_H(
    measured: np.ndarray, p0: gtsam.Pose2, p1: gtsam.Pose2, H: list[np.ndarray]
) -> np.ndarray:
    """measured: 1x1 array"""
    result = h(p0, p1) - measured
    if H is not None:
        H[0] = numericalDerivative21(h, p0, p1)
        H[1] = numericalDerivative22(h, p0, p1)
    return result


def factor(
    measured: np.ndarray,
    model: SharedNoiseModel,
    p0_key: int,
    p1_key: int,
) -> gtsam.NoiseModelFactor:
    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        p0: gtsam.Pose2 = v.atPose2(this.keys()[0])
        p1: gtsam.Pose2 = v.atPose2(this.keys()[1])
        return h_H(measured, p0, p1, H)

    return gtsam.CustomFactor(
        model,
        gtsam.KeyVector([p0_key, p1_key]),  # type:ignore
        error_func,
    )
