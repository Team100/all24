# pylint: disable=R0902,R0903,W0212

from typing_extensions import override
from wpimath.geometry import Rotation3d, Rotation2d
from app.network.network_protocol import (
    Blip24,
    Blip25,
    Blip25Receiver,
    Blip25Sender,
    BlipSender,
    DoubleSender,
    GyroReceiver,
    Network,
    NoteSender,
    OdometryReceiver,
    PoseEstimate25,
    PoseSender,
)
from app.pose_estimator.swerve_module_position import SwerveModulePositions


class FakeDoubleSender(DoubleSender):
    def __init__(self, doubles: list[float]) -> None:
        self.doubles = doubles

    @override
    def send(self, val: float, delay_us: int) -> None:
        self.doubles.append(val)


class FakeBlipSender(BlipSender):
    def __init__(self, blips: list[Blip24]) -> None:
        self.blips = blips

    @override
    def send(self, val: list[Blip24], delay_us: int) -> None:
        self.blips.extend(val)


class FakeNoteSender(NoteSender):
    def __init__(self, notes: list[Rotation3d]) -> None:
        self.notes = notes

    @override
    def send(self, val: list[Rotation3d], delay_us: int) -> None:
        self.notes.extend(val)


class FakeBlip25Sender(Blip25Sender):
    def __init__(self, blips: list[Blip25]) -> None:
        self.blips = blips

    @override
    def send(self, val: list[Blip25], delay_us: int) -> None:
        self.blips.extend(val)


class FakeBlip25Receiver(Blip25Receiver):
    def __init__(self, name: str, net: "FakeNetwork") -> None:
        self.name = name
        self.net = net

    @override
    def get(self) -> list[tuple[int, list[Blip25]]]:
        return self.net.received_blip25s[self.name]


class FakePoseSender(PoseSender):
    def __init__(self, name: str, net: "FakeNetwork") -> None:
        self.name = name
        self.net = net

    @override
    def send(self, val: PoseEstimate25, delay_us: int) -> None:
        self.net.estimate = val


class FakeOdometryReceiver(OdometryReceiver):
    def __init__(self, name: str, net: "FakeNetwork") -> None:
        self.name = name
        self.net = net

    def get(self) -> list[tuple[int, SwerveModulePositions]]:
        return self.net.received_positions


class FakeGyroReceiver(GyroReceiver):
    def __init__(self, name: str, net: "FakeNetwork") -> None:
        self.name = name
        self.net = net

    def get(self) -> list[tuple[int, Rotation2d]]:
        return self.net.received_yaw


class FakeNetwork(Network):
    def __init__(self) -> None:
        self.doubles: dict[str, list[float]] = {}
        self.blips: dict[str, list[Blip24]] = {}
        self.blip25s: dict[str, list[Blip25]] = {}
        # key: camera, list of updates, each update is tuple (timestamp, list of blips)
        self.received_blip25s: dict[str, list[tuple[int, list[Blip25]]]] = {}
        self.notes: dict[str, list[Rotation3d]] = {}
        self.estimate: PoseEstimate25
        self.received_positions: list[tuple[int, SwerveModulePositions]] = []
        self.received_yaw: list[tuple[int, Rotation2d]] = []

    @override
    def get_double_sender(self, name: str) -> DoubleSender:
        if name not in self.doubles:
            self.doubles[name] = []
        return FakeDoubleSender(self.doubles[name])

    @override
    def get_blip_sender(self, name: str) -> BlipSender:
        if name not in self.blips:
            self.blips[name] = []
        return FakeBlipSender(self.blips[name])

    @override
    def get_note_sender(self, name: str) -> NoteSender:
        if name not in self.notes:
            self.notes[name] = []
        return FakeNoteSender(self.notes[name])

    @override
    def get_blip25_sender(self, name: str) -> Blip25Sender:
        if name not in self.blip25s:
            self.blip25s[name] = []
        return FakeBlip25Sender(self.blip25s[name])

    @override
    def get_blip25_receiver(self, name: str) -> Blip25Receiver:
        if name not in self.received_blip25s:
            self.received_blip25s[name] = []
        return FakeBlip25Receiver(name, self)

    @override
    def get_pose_sender(self, name: str) -> PoseSender:
        return FakePoseSender(name, self)

    @override
    def get_odometry_receiver(self, name: str) -> OdometryReceiver:
        return FakeOdometryReceiver(name, self)

    @override
    def get_gyro_receiver(self, name: str) -> GyroReceiver:
        return FakeGyroReceiver(name, self)

    @override
    def flush(self) -> None:
        pass
