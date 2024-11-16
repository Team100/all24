import unittest

import ntcore

from app.config.identity import Identity
from app.network.fake_network import FakeNetwork
from app.network.network_protocol import Blip25
from app.network.real_network import RealNetwork
from app.pose_estimator.estimate import Estimate
from tests.pose_estimator.nt_sim import NTSim


class NTSimEstTest(unittest.TestCase):
    def test_sender_receiver(self) -> None:
        net = RealNetwork(Identity.UNKNOWN)
        sim = NTSim(net)
        est = Estimate()
        