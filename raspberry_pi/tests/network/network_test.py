import unittest

from app.network.fake_network import FakeNetwork, doubles


class NetworkTest(unittest.TestCase):
    def test_send(self) -> None:
        network = FakeNetwork()
        sender = network.get_double_sender("foo")
        sender.send(1.0, 0)
        self.assertEqual(1.0, doubles["foo"][0])
