"""Exercise the network tables simulator."""

import unittest

import ntcore

from app.config.identity import Identity
from app.network.structs import Blip25
from app.network.network import Network
from tests.pose_estimator.nt_sim import NTSim


class NTSimTest(unittest.TestCase):

    def test_real_nt_sim(self) -> None:
        print()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        sub = inst.getStructArrayTopic("blip25", Blip25).subscribe([])
        net = Network(Identity.UNKNOWN)
        sim = NTSim(net)
        for _ in range(50):
            sim.step(0.02)
            print(sub.readQueue())
