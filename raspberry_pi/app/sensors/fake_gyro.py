""" Fake gyro for testing. """

# pylint: disable=too-few-public-methods
import math
import time

from app.config.identity import Identity
from app.network.network import Network
from app.sensors.gyro_protocol import Gyro


class FakeGyro(Gyro):
    def __init__(self, identity: Identity, network: Network) -> None:
        path = "gyro/" + identity.value
        self._theta = network.get_double_sender(path + "/omega")
        self._omega = network.get_double_sender(path + "/theta")

    def sample(self) -> None:
        # some fake data just to see if it moves
        t: float = time.time()
        self._theta.send(math.sin(t), 0)
        self._omega.send(math.cos(t), 0)
