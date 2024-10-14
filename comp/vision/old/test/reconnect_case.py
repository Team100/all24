# This is the test case that illustrates client zombie
# on network disruption.
#
# I logged a ticket for this case:
# https://github.com/wpilibsuite/allwpilib/issues/5146

import time
from ntcore import NetworkTableInstance

inst = NetworkTableInstance.getDefault()
inst.startClient4("myclient")
inst.setServer("10.1.0.2")
pub = inst.getTable("mytable").getIntegerTopic("mytopic").publish()
counter = 0
while True:
    pub.set(counter)
    counter += 1
    if counter > 10:
        print("reconnecting...")
        counter = 0
        inst.stopClient()
        inst.startClient4("myclient")
    time.sleep(0.5)
