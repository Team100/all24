""" This is a wrapper for network tables.
"""

# pylint: disable=R0902,R0903,W0212

import dataclasses
import ntcore
from wpimath.geometry import Transform3d
from wpiutil import wpistruct
from app.config.identity import Identity


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


class DoubleSender:
    def __init__(self, pub: ntcore.DoublePublisher) -> None:
        self.pub: ntcore.DoublePublisher = pub

    def send(self, val: float, delay_us: int) -> None:
        self.pub.set(val, int(ntcore._now() - delay_us))


class BlipSender:
    def __init__(self, pub: ntcore.StructArrayPublisher) -> None:
        self.pub: ntcore.StructArrayPublisher = pub

    def send(self, val: list[Blip24], delay_us: int) -> None:
        self.pub.set(val, int(ntcore._now() - delay_us))


class Network:
    def __init__(self, identity: Identity, camera_num: int) -> None:
        # TODO: use identity.name instead
        # TODO: make Network work for gyro, not just vision
        self._serial: str = identity.value
        self._inst: ntcore.NetworkTableInstance = (
            ntcore.NetworkTableInstance.getDefault()
        )
        self._inst.startClient4("tag_finder24")

        ntcore._now()

        # roboRio address. windows machines can impersonate this for simulation.
        # also localhost for testing
        self._inst.setServer(["10.1.0.2", "127.0.0.1"])
       


    def get_double_sender(self, name: str) -> DoubleSender:
        return DoubleSender(self._inst.getDoubleTopic(name).publish())

    def get_blip_sender(self, name: str) -> BlipSender:
        return BlipSender(self._inst.getStructArrayTopic(name, Blip24).publish())

    def flush(self) -> None:
        self._inst.flush()
