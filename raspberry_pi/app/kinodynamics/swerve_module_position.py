""" SwerveModulePosition100

The classes here should mirror the Java ones.
"""

import dataclasses

from wpimath.geometry import Rotation2d
from wpiutil import wpistruct


@wpistruct.make_wpistruct
@dataclasses.dataclass
class OptionalRotation2d:
    present: bool
    value: Rotation2d


@wpistruct.make_wpistruct
@dataclasses.dataclass
class SwerveModulePosition100:
    distance_m: float
    angle: OptionalRotation2d


@wpistruct.make_wpistruct
@dataclasses.dataclass
class SwerveModulePositions:
    """See SwerveModulePositions.java"""

    front_left: SwerveModulePosition100
    front_right: SwerveModulePosition100
    rear_left: SwerveModulePosition100
    rear_right: SwerveModulePosition100
