# pylint: disable=E1101
import gtsam
import math
from wpimath.geometry import (
    Pose3d,
    Translation3d,
    Rotation3d,
    Pose2d,
    Translation2d,
    Rotation2d,
)

from app.network.structs import Cal3DS2

# discrete time step is 20 ms
TIME_STEP_US = 20000


def discrete(timestamp_us: int) -> int:
    # print("TIMESTAMP_US",timestamp_us)
    """Discretize time at 50 Hz"""
    return math.ceil(timestamp_us / TIME_STEP_US) * TIME_STEP_US


def pose3_to_pose3d(pose3: gtsam.Pose3) -> Pose3d:
    """GTSAM Pose3 is (Rot3, Point3)
    Point3 is just a Vector, i.e. just an np array.
    Rot3 is either a quaternion or an SO3, depending on a compile-time
    flag, SO3 by default, which is represented by a 3x3 matrix.

    WPI Pose3d is (Translation3d, Rotation3d)
    Translation3d is three scalars (in meters).
    Rotation3d is a quaternion.

    So we can supply the 3x3 to WPI or we can get GTSAM to convert to a quaternion.
    gtsam does the quaternion conversion in Eigen, see Quaternion.h:809, it's
    the Shoemake algorithm.  WPI does quaternion conversion in Rotation3d.h:111,
    differently than Eigen.  I guess I'd rather use the WPI code since it's
    easier to find, easier to read, and easier to complain about if required.
    A quaternion in gtsam is an Eigen quaternion typedef; the
    coeffs() method is wrapped in python; it returns a 4x1 matrix
    in x,y,z,w order.
    in WPI it's four scalars, you specify them in the construtor."""
    return Pose3d(
        Translation3d(pose3.translation()), Rotation3d(pose3.rotation().matrix())
    )


def pose2_to_pose2d(pose2: gtsam.Pose2) -> Pose2d:
    """GTSAM representation of Pose2 is Rot2 and Point2 (which is just nparray).
    Rot2 is cos and sin.
    WPI Rotation2d is also cos and sin.
    """
    return Pose2d(
        Translation2d(pose2.translation()),
        Rotation2d(pose2.rotation().c(), pose2.rotation().s()),
    )


def pose2d_to_pose2(pose2d: Pose2d) -> gtsam.Pose2:
    return gtsam.Pose2(pose2d.x, pose2d.y, pose2d.rotation().radians())


def to_cal(gc: gtsam.Cal3DS2) -> Cal3DS2:
    return Cal3DS2(gc.fx(), gc.fy(), gc.skew(), gc.px(), gc.py(), gc.k1(), gc.k2())
    # v = gc.vector()
    # return Cal3DS2(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8])


def make_smoother(lag_s: float) -> gtsam.BatchFixedLagSmoother:
    """lag_s: length of the smoother lag window in seconds."""
    # experimenting with the size of the lag buffer.
    # the python odometry factor is intolerably slow
    # but the native one is quite fast.
    # i'm not sure what sort of window we really need; maybe
    # just long enough to span periods of blindness, so,
    # like a second or two?
    # a long window is VERY SLOW, so try very short windows
    # just long enough to catch a single vision update.
    lag_us = lag_s * 1e6
    lm_params = gtsam.LevenbergMarquardtParams()
    return gtsam.BatchFixedLagSmoother(lag_us, lm_params)
