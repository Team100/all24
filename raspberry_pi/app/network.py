"""
This is a wrapper for network tables.
"""

# pylint: disable=R0902

import dataclasses
import ntcore
from wpimath.geometry import Transform3d
from wpiutil import wpistruct
from app.config.identity import Identity


@wpistruct.make_wpistruct
@dataclasses.dataclass
class Blip24:
    """AprilTag target pose used in 2024"""
    id: int
    pose: Transform3d

@wpistruct.make_wpistruct
@dataclasses.dataclass
class Blip25:
    """AprilTag target for 2025, includes pixel coordinates, for GTSAM."""
    id: int

    pose: Transform3d

class Network:
    def __init__(self, identity: Identity, camera_num: int) -> None:
        # TODO: use identity.name instead
        # TODO: make Network work for gyro, not just vision
        self.serial: str = identity.value
        self.inst: ntcore.NetworkTableInstance = (
            ntcore.NetworkTableInstance.getDefault()
        )
        self.inst.startClient4("tag_finder24")

        ntcore._now()

        # roboRio address. windows machines can impersonate this for simulation.
        # also localhost for testing
        self.inst.setServer(["10.1.0.2", "127.0.0.1"])
        topic_name: str = "vision/" + self.serial + str(camera_num)
        self.vision_capture_time_ms: ntcore.DoublePublisher = self.inst.getDoubleTopic(
            topic_name + "/capture_time_ms"
        ).publish()
        self.vision_image_age_ms = self.inst.getDoubleTopic(
            topic_name + "/image_age_ms"
        ).publish()
        self.vision_total_time_ms = self.inst.getDoubleTopic(
            topic_name + "/total_time_ms"
        ).publish()
        self.vision_detect_time_ms = self.inst.getDoubleTopic(
            topic_name + "/detect_time_ms"
        ).publish()

        self._gyro_yaw = self.inst.getDoubleTopic(topic_name + "/gyro_yaw").publish()
        self._gyro_rate = self.inst.getDoubleTopic(topic_name + "/gyro_rate").publish()

        # work around https://github.com/robotpy/mostrobotpy/issues/60
        self.inst.getStructTopic("bugfix", Blip24).publish().set(
            Blip24(0, Transform3d())
        )

        # blip array topic
        self.vision_nt_struct = self.inst.getStructArrayTopic(
            topic_name + "/blips", Blip24
        ).publish()

    def set_gyro_yaw(self, value: float, delay_us: int) -> None:
        self._gyro_yaw.set(value, ntcore._now() - delay_us)

    def set_gyro_rate(self, value: float, delay_us: int) -> None:
        self._gyro_rate.set(value, ntcore._now() - delay_us)

    def flush(self) -> None:
        self.inst.flush()
