#!/usr/bin/env python3

import time
import ntcore

if __name__ == "__main__":
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable("datatable")
    xSub = table.getDoubleTopic("x").publish(ntcore.PubSubOptions(sendAll=True))
    ySub = table.getDoubleTopic("y").publish()
    inst.setServer("localhost")
    inst.startClient4("example client")
    # x = 0
    # y = 0

    while True:
        time.sleep(1)
        xSub.set(1.0)
        ySub.set(1.0)

        # xSub.set(x)
        # ySub.set(y)
        # x += 1
        # y += 1
