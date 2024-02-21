#!/usr/bin/env python3

import time
import ntcore
from wpimath.geometry import Rotation3d

if __name__ == "__main__":
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable("datatable")
    pub = inst.getStructArrayTopic("rot", Rotation3d).publish(
        ntcore.PubSubOptions(keepDuplicates=True)
    )
    inst.setServer("localhost")
    inst.startClient4("example client")

    while True:
        time.sleep(1)
        print("send")
        rots = [Rotation3d(1, 2, 3), Rotation3d(4, 5, 6)]
        pub.set(rots)
