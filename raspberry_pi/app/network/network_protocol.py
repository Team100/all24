""" Interface for Network Tables-like things. """

# pylint: disable=R0902,R0903,W0212


import dataclasses
from typing import Protocol

from wpimath.geometry import Rotation3d, Transform3d
from wpiutil import wpistruct


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class Blip24:
    """AprilTag target pose used in 2024"""

    id: int
    pose: Transform3d


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class Blip25:
    """AprilTag target for 2025, includes pixel coordinates, for GTSAM."""

    id: int
    pose: Transform3d


class DoubleSender(Protocol):
    def send(self, val: float, delay_us: int) -> None: ...


class BlipSender(Protocol):
    def send(self, val: list[Blip24], delay_us: int) -> None: ...


class NoteSender(Protocol):
    def send(self, val: list[Rotation3d], delay_us: int) -> None: ...


class Network(Protocol):
    def get_double_sender(self, name: str) -> DoubleSender: ...
    def get_blip_sender(self, name: str) -> BlipSender: ...
    def get_note_sender(self, name: str) -> NoteSender: ...
    def flush(self) -> None: ...
