"""Factory for odometry factors"""

# pylint: disable=C0103,E0611

from typing import Callable

import gtsam
import numpy as np
from gtsam import (
    BetweenFactorDouble,
    BetweenFactorPose2,
    CustomFactor,
    FixedLagSmoother,
    ISAM2GaussNewtonParams,
    KeyVector,
    NonlinearFactor,
    NonlinearFactorGraph,
    Point2,
    Point3,
    Pose2,
    Pose3,
    Rot2,
    Rot3,
    Values,
)
from gtsam.noiseModel import Base as SharedNoiseModel
from gtsam.noiseModel import Diagonal
from gtsam.symbol_shorthand import X  # robot pose
from wpimath.geometry import Twist2d

from app.pose_estimator.numerical_derivative import (
    numericalDerivative21,
    numericalDerivative22,
)


def odometry_h_fn() -> Callable[[Pose2, Pose2], np.ndarray]:
    def h(p0: Pose2, p1: Pose2) -> np.ndarray:
        return p0.logmap(p1)

    return h


def odo_H(measured, p0, p1, H):
    h = odometry_h_fn()
    result = h(p0, p1) - measured
    if H is not None:
        H[0] = numericalDerivative21(h, p0, p1)
        H[1] = numericalDerivative22(h, p0, p1)
    return result


def odometry_factor(
    t: Twist2d,
    model: SharedNoiseModel,
    p0_key: int,
    p1_key: int,
) -> NonlinearFactor:
    measured = np.array([t.dx, t.dy, t.dtheta])

    def error_func(this: CustomFactor, v: Values, H: list[np.ndarray]) -> np.ndarray:
        p0 = v.atPose2(this.keys()[0])
        p1 = v.atPose2(this.keys()[1])
        result = odo_H(measured, p0, p1, H)

        return result

    return CustomFactor(model, KeyVector([p0_key, p1_key]), error_func)
