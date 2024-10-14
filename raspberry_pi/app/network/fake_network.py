# pylint: disable=R0902,R0903,W0212

from typing_extensions import override
from wpimath.geometry import Rotation3d

from app.network.network_protocol import (Blip24, BlipSender, DoubleSender,
                                          Network, NoteSender)

doubles: dict[str, list[float]] = {}
blips: dict[str, list[Blip24]] = {}
notes: dict[str, list[Rotation3d]] = {}


class FakeDoubleSender(DoubleSender):
    def __init__(self, name: str) -> None:
        self.name = name
        if name not in doubles:
            doubles[name] = []

    @override
    def send(self, val: float, delay_us: int) -> None:
        print(self.name, ": ", val)
        doubles[self.name].append(val)


class FakeBlipSender(BlipSender):
    def __init__(self, name: str) -> None:
        self.name = name
        if name not in blips:
            blips[name] = []

    @override
    def send(self, val: list[Blip24], delay_us: int) -> None:
        blips[self.name].extend(val)


class FakeNoteSender(NoteSender):
    def __init__(self, name: str) -> None:
        self.name = name
        if name not in notes:
            notes[name] = []

    @override
    def send(self, val: list[Rotation3d], delay_us: int) -> None:
        notes[self.name].extend(val)


class FakeNetwork(Network):

    @override
    def get_double_sender(self, name: str) -> DoubleSender:
        return FakeDoubleSender(name)

    @override
    def get_blip_sender(self, name: str) -> BlipSender:
        return FakeBlipSender(name)

    @override
    def get_note_sender(self, name: str) -> NoteSender:
        return FakeNoteSender(name)

    @override
    def flush(self) -> None:
        pass
