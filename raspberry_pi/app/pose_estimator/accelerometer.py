"""Factory for accelerometer factors.

The GTSAM version of this idea is kind of buried in the
IMUFactor.

See https://github.com/borglab/gtsam/blob/develop/doc/ImuFactor.pdf

This is simpler.

TODO: measure some accelerometers to find a reasonable noise model.

The Redux guy says his thing is an "instantaneous" measurement but the
reality is that all sensors do some sort of filtering, so there's some
investigation to do here.

For example, the LSM6DSOX has two stages, an analog stage to prevent
ADC aliasing which probably runs at 1.5khz, and a digital stage that runs
at ODR/2 (Nyquist) or slower.

Eeshwar says they do 1ms which kind of implies a 1khz filter, so when
we get CAN packets (at 50 or 100hz) they are only doing points near the
CAN output, not averaging or filtering in a useful way at all.

In that case, we should supplement the Redux box with something like
the LSM6DSOX with an ODR that matches our actual accel need.
"""

# pylint: disable=C0103,E0611,E1101,R0913

import gtsam
import numpy as np
from gtsam.noiseModel import Base as SharedNoiseModel

from app.pose_estimator.numerical_derivative import (
    numericalDerivative31,
    numericalDerivative32,
    numericalDerivative33,
)


def h(
    p0: gtsam.Pose2, p1: gtsam.Pose2, p2: gtsam.Pose2, dt1: float, dt2: float
) -> np.ndarray:
    """Estimated tangential acceleration at p2.
    Computes the second-order backward finite difference, in tangent space.
    TODO: something better than the dt's here?
    In 2d, the translation velocity and angular velocity are always perpendicular
    so the coriolis force is always -2*omega*v"""
    twist1 = p0.logmap(p1)
    v1 = twist1 / dt1
    twist2 = p1.logmap(p2)
    v2 = twist2 / dt2
    # the translational velocity in the manifold
    v = np.copy(v2)
    v[2] = 0
    # the rotational velocity in the manifold
    omega = np.copy(v2)
    omega[0] = 0
    omega[1] = 0
    # coriolis accel
    coriolis = -2.0 * np.cross(omega, v)
    inertial = (v2 - v1) / dt2
    return coriolis + inertial


def h_H(
    measured: np.ndarray,
    p0: gtsam.Pose2,
    p1: gtsam.Pose2,
    p2: gtsam.Pose2,
    dt1: float,
    dt2: float,
    H: list[np.ndarray],
):
    """Error function including Jacobians."""
    result = h(p0, p1, p2, dt1, dt2) - measured
    if H is not None:
        H[0] = numericalDerivative31(lambda x, y, z: h(x, y, z, dt1, dt2), p0, p1, p2)
        H[1] = numericalDerivative32(lambda x, y, z: h(x, y, z, dt1, dt2), p0, p1, p2)
        H[2] = numericalDerivative33(lambda x, y, z: h(x, y, z, dt1, dt2), p0, p1, p2)
    return result


def factor(
    x: float,
    y: float,
    dt1: float,
    dt2: float,
    model: SharedNoiseModel,
    p0_key: gtsam.Symbol,
    p1_key: gtsam.Symbol,
    p2_key: gtsam.Symbol,
) -> gtsam.NonlinearFactor:
    # TODO: something other than dt1 and dt2?
    # this is the robot-frame acceleration vector.
    measured = np.array([x, y, 0])

    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        p0: gtsam.Pose2 = v.atPose2(this.keys()[0])
        p1: gtsam.Pose2 = v.atPose2(this.keys()[1])
        p2: gtsam.Pose2 = v.atPose2(this.keys()[2])
        return h_H(measured, p0, p1, p2, dt1, dt2, H)

    return gtsam.CustomFactor(
        model, gtsam.KeyVector([p0_key, p1_key, p2_key]), error_func
    )
