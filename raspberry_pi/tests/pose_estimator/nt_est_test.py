import unittest

import ntcore

from app.config.identity import Identity
from app.network.fake_network import FakeNetwork
from app.network.network_protocol import Blip25
from app.network.real_network import RealNetwork
from app.pose_estimator.estimate import Estimate
from app.pose_estimator.nt_estimate import NTEstimate
from tests.pose_estimator.nt_sim import NTSim


class NTEstTest(unittest.TestCase):
    def test_real_nt_est(self) -> None:
        net = RealNetwork(Identity.UNKNOWN)
        est = NTEstimate(net)

    def test_fake_nt_est(self) -> None:
        net = FakeNetwork()
        est = NTEstimate(net)
