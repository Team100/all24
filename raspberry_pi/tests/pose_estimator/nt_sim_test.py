import unittest

import ntcore

from app.config.identity import Identity
from app.network.fake_network import FakeNetwork
from app.network.network_protocol import Blip25
from app.network.real_network import RealNetwork
from tests.pose_estimator.nt_sim import NTSim


class NTSimTest(unittest.TestCase):

    def test_real_nt_sim(self) -> None:
        print()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        sub = inst.getStructArrayTopic("foo", Blip25).subscribe([])
        net = RealNetwork(Identity.UNKNOWN)
        sim = NTSim(net)
        for _ in range(50):
            sim.step(0.02)
            print(sub.readQueue())

    def test_fake_nt_sim(self) -> None:
        net = FakeNetwork()
        sim = NTSim(net)
        for _ in range(50):
            sim.step(0.02)
            print(net.blip25s)
