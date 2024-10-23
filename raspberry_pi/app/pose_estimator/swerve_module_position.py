""" SwerveModulePosition100"""

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
