""" Fake gyro for testing. """

# pylint: disable=too-few-public-methods

from app.gyro import Gyro
from app.network import Network


class FakeGyro(Gyro):
    def __init__(self, network: Network) -> None:
        self.network = network

    def sample(self) -> None:
        self.network.set_gyro_yaw(0, 0)
        self.network.set_gyro_rate(0, 0)
