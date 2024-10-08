""" Fake gyro for testing. """

# pylint: disable=too-few-public-methods
import math
import time
from app.sensors.gyro_protocol import Gyro
from app.localization.network import Network


class FakeGyro(Gyro):
    def __init__(self, network: Network) -> None:
        self.network = network

    def sample(self) -> None:
        # some fake data just to see if it moves
        t: float = time.time()
        self.network.set_gyro_yaw(math.sin(t), 0)
        self.network.set_gyro_rate(math.cos(t), 0)

        # self.network.set_gyro_yaw(0, 0)
        # self.network.set_gyro_rate(0, 0)
