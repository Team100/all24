# pylint: disable=consider-using-from-import,invalid-name,no-name-in-module,no-member,missing-function-docstring,too-many-locals
"""
A structure-from-motion example with landmarks, default function arguments give
 - The landmarks form a 10 meter cube
 - The robot rotates around the landmarks, always facing towards the cube
Passing function argument allows to specificy an initial position, a pose increment and step count.
"""
import math

from gtsam import Point3, Pose3, Rot3


def createPoints() -> list[Point3]:
    """
    Create the set of ground-truth landmarks
    """
    return [
        Point3(10.0, 10.0, 10.0),
        Point3(-10.0, 10.0, 10.0),
        Point3(-10.0, -10.0, 10.0),
        Point3(10.0, -10.0, 10.0),
        Point3(10.0, 10.0, -10.0),
        Point3(-10.0, 10.0, -10.0),
        Point3(-10.0, -10.0, -10.0),
        Point3(10.0, -10.0, -10.0),
    ]


def createPoses(
    init: Pose3 = Pose3(Rot3.Ypr(math.pi / 2, 0, -math.pi / 2), Point3(30, 0, 0)),
    delta: Pose3 = Pose3(
        Rot3.Ypr(0, -math.pi / 4, 0),
        Point3(math.sin(math.pi / 4) * 30, 0, 30 * (1 - math.sin(math.pi / 4))),
    ),
    steps: int = 8,
) -> list[Pose3]:
    """
    Create the set of ground-truth poses
    Default values give a circular trajectory,
    radius 30 at pi/4 intervals, always facing the circle center
    """
    poses: list[Pose3] = []
    poses.append(init)
    for i in range(steps):
        poses.append(poses[i - 1].compose(delta))
    return poses
