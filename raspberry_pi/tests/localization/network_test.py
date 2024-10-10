import unittest

import ntcore

from app.config.identity import Identity
from app.localization.network import Network


class NetworkTest(unittest.TestCase):
    def test_send(self) -> None:
        inst = ntcore.NetworkTableInstance.getDefault()
        network: Network = Network(Identity.UNKNOWN, 0)
        sender = network.get_double_sender("foo")
        sender.send(1.0, 0)
        self.assertEqual(1.0, inst.getDoubleTopic("foo").getEntry(-1).get())
