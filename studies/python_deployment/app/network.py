""" This is a wrapper for network tables.
"""

import ntcore

from app.identity import Identity


class Network:
    def __init__(self, identity: Identity) -> None:
        # TODO: use identity.name instead
        self.serial: str = identity.value
        self.inst: ntcore.NetworkTableInstance = (
            ntcore.NetworkTableInstance.getDefault()
        )
        self.inst.startClient4("tag_finder24")

        # roboRio address. windows machines can impersonate this for simulation.
        self.inst.setServer("10.1.0.2")
        topic_name: str = "vision/" + self.serial
        self.vision_capture_time_ms: ntcore.DoublePublisher = self.inst.getDoubleTopic(
            topic_name + "/capture_time_ms"
        ).publish()

    def log_capture_time(self, ms: float) -> None:
        self.vision_capture_time_ms.set(ms)
