""" This is a wrapper for network tables. """

# pylint: disable=R0902,R0903,W0212

# github workflow crashes in ntcore, bleah
from typing import cast

import ntcore
from wpimath.geometry import Pose2d, Rotation2d, Rotation3d
from wpiutil import wpistruct

from app.config.identity import Identity
from app.kinodynamics.swerve_module_position import SwerveModulePositions
from app.network.structs import (
    Blip24,
    Blip25,
    CameraCalibration,
    PoseEstimate25,
)

# global singleton
start_time_us = ntcore._now()


class DoubleSender:
    def __init__(self, pub: ntcore.DoublePublisher) -> None:
        self.pub = pub

    def send(self, val: float, delay_us: int) -> None:
        self.pub.set(val, int(ntcore._now() - delay_us))


class BlipSender:
    def __init__(self, pub: ntcore.StructArrayPublisher) -> None:
        self.pub = pub

    def send(self, val: list[Blip24], delay_us: int) -> None:
        self.pub.set(val, int(ntcore._now() - delay_us))


class NoteSender:
    def __init__(self, pub: ntcore.StructArrayPublisher) -> None:
        self.pub = pub

    def send(self, val: list[Rotation3d], delay_us: int) -> None:
        self.pub.set(val, int(ntcore._now() - delay_us))


class Blip25Sender:
    def __init__(self, pub: ntcore.StructArrayPublisher) -> None:
        self.pub = pub

    def send(self, val: list[Blip25], delay_us: int) -> None:
        timestamp = int(ntcore._now() - delay_us)
        self.pub.set(val, timestamp)


class Blip25Receiver:
    """Receive the list of tuples (timetamp, list[blip]) seen in a single frame"""

    def __init__(
        self,
        name: str,
        inst: ntcore.NetworkTableInstance,
    ) -> None:
        # print("RealBlip25Receiver.__init__() name ", name)
        self.name = name
        self.poller = ntcore.NetworkTableListenerPoller(inst)
        # need to hang on to this reference :-(
        self.msub = ntcore.MultiSubscriber(
            inst, [name], ntcore.PubSubOptions(keepDuplicates=True)
        )
        self.poller.addListener(self.msub, ntcore.EventFlags.kValueAll)
        # self.poller.addListener([""], ntcore.EventFlags.kValueAll)

        # print("RealBlip25Receiver.__init__() start_time_us ", self.start_time_us)

    def get(self) -> list[tuple[int, list[Blip25]]]:
        """(timestamp_us, tag id, blip)
        The timestamp is referenced to the "now" value at
        construction, so that the number isn't too large.
        """
        result: list[tuple[int, list[Blip25]]] = []
        # print("RealBlip25Receiver.get()")
        # see NotePosition24ArrayListener for example
        queue: list = self.poller.readQueue()
        # print("RealBlip25Receiver.get() queue length ", len(queue))
        for event in queue:
            value_event_data = cast(ntcore.ValueEventData, event.data)

            # name = value_event_data.topic.getName()
            # TODO: redo the key scheme
            # camera_id = int(name.split("/")[0])

            nt_value: ntcore.Value = value_event_data.value

            # server time is always 1.  ???
            server_time_us = nt_value.server_time()
            # print("RealBlip25Receiver.get() server time ", server_time_us)
            time_us = nt_value.time() - start_time_us
            # print("RealBlip25Receiver.get() time ", time_us)

            frame: list[Blip25] = []
            raw_array: bytes = cast(bytes, nt_value.getRaw())
            item_size = wpistruct.getSize(Blip25)
            raw_item_array = [
                raw_array[i : i + item_size]
                for i in range(0, len(raw_array), item_size)
            ]
            for raw_item in raw_item_array:
                blip: Blip25 = wpistruct.unpack(Blip25, raw_item)
                # print(blip)
                frame.append(blip)
            result.append((time_us, frame))
        return result


class PoseSender:
    def __init__(self, pub: ntcore.StructPublisher) -> None:
        self.pub = pub

    def send(self, val: PoseEstimate25, delay_us: int) -> None:
        self.pub.set(val, int(ntcore._now() - delay_us))


class OdometrySender:
    def __init__(self, pub: ntcore.StructPublisher) -> None:
        self.pub = pub

    def send(self, val: SwerveModulePositions, delay_us: int) -> None:
        self.pub.set(val, int(ntcore._now() - delay_us))


class OdometryReceiver:
    """Receive a list of tuples (timestamp, positions)"""

    def __init__(
        self,
        name: str,
        inst: ntcore.NetworkTableInstance,
    ) -> None:
        self.name = name
        self.poller = ntcore.NetworkTableListenerPoller(inst)
        # need to hang on to this reference :-(
        self.msub = ntcore.MultiSubscriber(
            inst, [name], ntcore.PubSubOptions(keepDuplicates=True)
        )
        self.poller.addListener(self.msub, ntcore.EventFlags.kValueAll)
        # self.poller.addListener([name], ntcore.EventFlags.kValueAll)

    def get(self) -> list[tuple[int, SwerveModulePositions]]:
        result: list[tuple[int, SwerveModulePositions]] = []
        # see NotePosition24ArrayListener for example
        queue: list = self.poller.readQueue()
        for event in queue:
            value_event_data = cast(ntcore.ValueEventData, event.data)
            nt_value: ntcore.Value = value_event_data.value
            time_us = nt_value.time() - start_time_us
            raw: bytes = cast(bytes, nt_value.getRaw())
            pos: SwerveModulePositions = wpistruct.unpack(SwerveModulePositions, raw)
            result.append((time_us, pos))
        return result


class GyroSender:
    def __init__(self, pub: ntcore.StructPublisher) -> None:
        self.pub = pub

    def send(self, val: Rotation2d, delay_us: int) -> None:
        self.pub.set(val, int(ntcore._now() - delay_us))


class GyroReceiver:
    """Receive a list of tuples (timestamp, yaw)"""

    def __init__(
        self,
        name: str,
        inst: ntcore.NetworkTableInstance,
    ) -> None:
        self.name = name
        self.poller = ntcore.NetworkTableListenerPoller(inst)
        # need to hang on to this reference :-(
        self.msub = ntcore.MultiSubscriber(
            inst, [name], ntcore.PubSubOptions(keepDuplicates=True)
        )
        self.poller.addListener(self.msub, ntcore.EventFlags.kValueAll)
        # self.poller.addListener([name], ntcore.EventFlags.kValueAll)

    def get(self) -> list[tuple[int, Rotation2d]]:
        result: list[tuple[int, Rotation2d]] = []
        queue: list = self.poller.readQueue()
        for event in queue:
            value_event_data = cast(ntcore.ValueEventData, event.data)
            nt_value: ntcore.Value = value_event_data.value
            time_us = nt_value.time() - start_time_us
            raw: bytes = cast(bytes, nt_value.getRaw())
            pos: Rotation2d = wpistruct.unpack(Rotation2d, raw)
            result.append((time_us, pos))
        return result


class CalibSender:
    def __init__(self, pub: ntcore.StructPublisher) -> None:
        self.pub = pub

    # TODO: we don't care about timestamp here so remove it
    def send(self, val: CameraCalibration, delay_us: int) -> None:
        self.pub.set(val, int(ntcore._now() - delay_us))


class PriorReceiver:
    """Force a pose."""

    def __init__(
        self,
        name: str,
        inst: ntcore.NetworkTableInstance,
    ) -> None:
        self.name = name
        self.poller = ntcore.NetworkTableListenerPoller(inst)
        # need to hang on to this reference :-(
        self.msub = ntcore.MultiSubscriber(
            inst, [name], ntcore.PubSubOptions(keepDuplicates=True)
        )
        self.poller.addListener(self.msub, ntcore.EventFlags.kValueAll)
        # self.poller.addListener([name], ntcore.EventFlags.kValueAll)

    def get(self) -> Pose2d | None:
        queue: list = self.poller.readQueue()
        # consecutive priors override each other so just take the last one
        if len(queue) == 0:
            return None
        event = queue[-1]

        value_event_data = cast(ntcore.ValueEventData, event.data)
        nt_value: ntcore.Value = value_event_data.value
        raw: bytes = cast(bytes, nt_value.getRaw())
        pose: Pose2d = wpistruct.unpack(Pose2d, raw)
        return pose


class Network:
    def __init__(self, identity: Identity) -> None:
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
        if identity == Identity.UNKNOWN:
            # vasili says this doesn't work, but i need it for testing.
            self._inst.setServer("localhost")
        else:
            # this works
            self._inst.setServer("10.1.0.2")

    def now(self) -> int:
        return ntcore._now() - start_time_us

    def get_double_sender(self, name: str) -> DoubleSender:
        return DoubleSender(self._inst.getDoubleTopic(name).publish())

    def get_blip_sender(self, name: str) -> BlipSender:
        return BlipSender(self._inst.getStructArrayTopic(name, Blip24).publish())

    def get_note_sender(self, name: str) -> NoteSender:
        return NoteSender(
            self._inst.getStructArrayTopic(name, Rotation3d).publish()
        )

    def get_blip25_sender(self, name: str) -> Blip25Sender:
        return Blip25Sender(self._inst.getStructArrayTopic(name, Blip25).publish())

    def get_blip25_receiver(self, name: str) -> Blip25Receiver:
        return Blip25Receiver(name, self._inst)

    def get_pose_sender(self, name: str) -> PoseSender:
        return PoseSender(self._inst.getStructTopic(name, PoseEstimate25).publish())

    def get_odometry_sender(self, name: str) -> OdometrySender:
        return OdometrySender(
            self._inst.getStructTopic(name, SwerveModulePositions).publish()
        )

    def get_odometry_receiver(self, name: str) -> OdometryReceiver:
        return OdometryReceiver(name, self._inst)

    def get_gyro_sender(self, name: str) -> GyroSender:
        return GyroSender(self._inst.getStructTopic(name, Rotation2d).publish())

    def get_gyro_receiver(self, name: str) -> GyroReceiver:
        return GyroReceiver(name, self._inst)

    def get_calib_sender(self, name: str) -> CalibSender:
        return CalibSender(
            self._inst.getStructTopic(name, CameraCalibration).publish()
        )

    def get_prior_receiver(self, name: str) -> PriorReceiver:
        return PriorReceiver(name, self._inst)

    def flush(self) -> None:
        self._inst.flush()
