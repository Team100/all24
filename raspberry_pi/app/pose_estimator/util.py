# pylint: disable=E1101
import gtsam
from wpimath.geometry import Pose3d, Translation3d, Rotation3d

from app.network.network_protocol import Cal3DS2


def pose3_to_pose3d(pose3: gtsam.Pose3) -> Pose3d:
    """GTSAM representation of Pose3 is a Point3 and a Rot3
    Point3 is just a Vector, i.e. just an np array.
    Rot3 is either a quaternion or an SO3, depending on a compile-time
    flag which indicates SO3 by default, which is represented by a 3x3 matrix.
    WPI representation of Translation3d is three scalars (in meters).
    WPI representation of Rotation3d is a quaternion.
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


def to_cal(gc: gtsam.Cal3DS2) -> Cal3DS2:
    v = gc.vector()
    return Cal3DS2(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8])
