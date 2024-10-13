import unittest

import ntcore

from app.config.identity import Identity
from app.network.real_network import RealNetwork


class NetworkTest(unittest.TestCase):
    def test_send(self) -> None:
        inst = ntcore.NetworkTableInstance.getDefault()
        network: RealNetwork = RealNetwork(Identity.UNKNOWN)
        sender = network.get_double_sender("foo")
        sender.send(1.0, 0)
        self.assertEqual(1.0, inst.getDoubleTopic("foo").getEntry(-1).get())
