""" Interface for Network Tables-like things. """

# pylint: disable=R0902,R0903,W0212,W2301

import dataclasses
from typing import Protocol

import numpy as np
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
    """AprilTag target for 2025, includes pixel coordinates.
    the struct concept only supports fixed-length records
    so i can't use an array here. it's ok because a tag only
    ever has exactly 4 corners; if one is occluded then the
    entire tag is unseen.
    """

    id: int
    llx: float  # lower left
    lly: float
    lrx: float  # lower right
    lry: float
    urx: float  # upper right
    ury: float
    ulx: float  # upper left
    uly: float

    def measurement(self) -> np.ndarray:
        """Concatenated corners, for GTSAM."""
        return np.array(
            [
                self.llx,
                self.lly,
                self.lrx,
                self.lry,
                self.urx,
                self.ury,
                self.ulx,
                self.uly,
            ]
        )


class DoubleSender(Protocol):
    def send(self, val: float, delay_us: int) -> None: ...


class BlipSender(Protocol):
    def send(self, val: list[Blip24], delay_us: int) -> None: ...


class NoteSender(Protocol):
    def send(self, val: list[Rotation3d], delay_us: int) -> None: ...


class Blip25Sender(Protocol):
    def send(self, val: list[Blip25], delay_us: int) -> None: ...


class Blip25Receiver(Protocol):
    def get(self) -> list[tuple[int, list[Blip25]]]:
        """Returns the list of tuples (timetamp, list[blip]) seen in a single frame"""
        ...


class Network(Protocol):
    def get_double_sender(self, name: str) -> DoubleSender: ...
    def get_blip_sender(self, name: str) -> BlipSender: ...
    def get_note_sender(self, name: str) -> NoteSender: ...
    def get_blip25_sender(self, name: str) -> Blip25Sender: ...
    def get_blip25_receiver(self, name: str) -> Blip25Receiver: ...
    def flush(self) -> None: ...
