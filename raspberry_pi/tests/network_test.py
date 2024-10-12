import unittest

from app.config.identity import Identity
from app.network import Network


class NetworkTest(unittest.TestCase):
    def test_identity(self) -> None:
        network: Network = Network(Identity.UNKNOWN, 0)
        network.set_gyro_yaw(0, 0)
        # TODO: add an assertion