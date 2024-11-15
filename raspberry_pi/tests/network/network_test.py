import unittest



import ntcore

from app.config.identity import Identity
from app.network.network import Network

class NetworkTest(unittest.TestCase):
    def test_send(self) -> None:
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        sub = inst.getDoubleTopic("foo").subscribe(0.0)

        network = Network(Identity.UNKNOWN)
        sender = network.get_double_sender("foo")
        sender.send(1.0, 0)
        self.assertEqual(1.0, sub.get())
