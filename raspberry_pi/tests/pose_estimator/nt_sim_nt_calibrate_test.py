"""Run the network tables simulator 
and network tables calibrator."""

# pylint: disable=W0212


import time
import unittest

import ntcore

from app.config.identity import Identity
from app.field.field_map import FieldMap
from app.network.structs import PoseEstimate25
from app.network.network import Network
from app.pose_estimator.nt_calibrate import NTCalibrate
from tests.pose_estimator.nt_sim import NTSim


class NTSimNTCalibrateTest(unittest.TestCase):
    def test_real_sender_receiver(self) -> None:
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        sub = inst.getStructTopic("pose", PoseEstimate25).subscribe(None)
        net = Network(Identity.UNKNOWN)
        sim = NTSim(net)
        field_map = FieldMap()
        est = NTCalibrate(field_map, net)
        for i in range(100):
            # TODO: i think for this test to work reliably i need to fully fake time.
            time.sleep(0.02)
            # print("\nSTEP", i)
            # publish some stuff
            sim.step(0.02)
            # collect it
            est.step()
            estimate = sub.get()
            # print("ESTIMATE", estimate)
