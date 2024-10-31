"""Factory for odometry factors.

This also serves as a template for other factors.  The pattern should always be
the same: an "h" function that performs the estimated measurement, an
"h_H" function that wraps it with numerical derivatives (to avoid confusion
with analytic Jacobians), and a "factor" factory that wraps that in a
CustomFactor.

There's some discussion relevant to these operations here:
https://groups.google.com/g/gtsam-users/c/c-BhH8mfqbo/m/IMk1RQ84AwAJ

NOTE! the native GTSAM BetweenFactor is much faster than the python
CustomFactor.  For a computation budget of 10ms, the python factor can
only handle about 45 values (i.e. a window of about 1 sec at 50 hz)
but the native factor can handle about 600 (i.e. a 12 sec window).

For production, we should use the native factor.  :-)
"""

# pylint: disable=C0103,E0611,E1101

import gtsam
import numpy as np
from gtsam.noiseModel import Base as SharedNoiseModel  # type:ignore
from wpimath.geometry import Twist2d, Pose2d

from app.pose_estimator.numerical_derivative import (
    numericalDerivative21,
    numericalDerivative22,
)


def h(p0: gtsam.Pose2, p1: gtsam.Pose2) -> np.ndarray:
    """Estimated tangential difference between p0 and p1.
    This is identical to the WPILib "Twist2d" idea."""
    return p0.logmap(p1)


def h_H(
    measured: np.ndarray, p0: gtsam.Pose2, p1: gtsam.Pose2, H: list[np.ndarray]
) -> np.ndarray:
    """Error function including Jacobians.
    Returns the difference between the measured and estimated tangent-space odometry."""
    result = h(p0, p1) - measured
    if H is not None:
        H[0] = numericalDerivative21(h, p0, p1)
        # H[1] = np.eye(3) # even slower than the numerical derivative!
        H[1] = numericalDerivative22(h, p0, p1)
    return result


def factorCustom(
    t: Twist2d,
    model: SharedNoiseModel,
    p0_key: int,
    p1_key: int,
) -> gtsam.NoiseModelFactor:
    """Uses a python CustomFactor."""
    measured = np.array([t.dx, t.dy, t.dtheta])

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


def factorNative(
    t: Twist2d,
    model: SharedNoiseModel,
    p0_key: int,
    p1_key: int,
) -> gtsam.NoiseModelFactor:
    """Uses the GTSAM BetweenFactor."""
    # the gtsam between factor uses a relative pose, not a twist.
    p = Pose2d().exp(t)
    gp = gtsam.Pose2(p.x, p.y, p.rotation().radians())
    return gtsam.BetweenFactorPose2(p0_key, p1_key, gp, model)


def factor(
    t: Twist2d,
    model: SharedNoiseModel,
    p0_key: int,
    p1_key: int,
) -> gtsam.NoiseModelFactor:
    """Factory for a factor implementing odometry using tangent-space measurements."""
    return factorNative(t, model, p0_key, p1_key)
    # return factorCustom(t, model, p0_key, p1_key)
