""" This is a wrapper for network tables.
"""

import ntcore


class Network:
    def __init__(self):
        self.serial = getserial()
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4("tag_finder24")

        # roboRio address. windows machines can impersonate this for simulation.
        self.inst.setServer("10.1.0.2")
        topic_name = "vision/" + self.serial
        self.vision_capture_time_ms = self.inst.getDoubleTopic(
            topic_name + "/capture_time_ms"
        ).publish()

    def log_capture_time(self, ms: float):
        self.vision_capture_time_ms.set(ms)


def getserial():
    with open("/proc/cpuinfo", "r", encoding="ascii") as cpuinfo:
        for line in cpuinfo:
            if line[0:6] == "Serial":
                return line[10:26]
    return ""
