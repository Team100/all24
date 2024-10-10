""" Chose a gyro implementation depending on platform. """

# pylint: disable=0903

from adafruit_platformdetect import Detector  # type:ignore
from app.config.identity import Identity
from app.sensors.gyro_protocol import Gyro
from app.sensors.fake_gyro import FakeGyro
from app.localization.network import Network
from app.sensors.real_gyro import RealGyro


class GyroFactory:
    @staticmethod
    def get(identity: Identity, network: Network) -> Gyro:
        detector = Detector()
        match detector.board.id:
            case "GENERIC_LINUX_PC":
                return FakeGyro(identity, network)
            case "RASPBERRY_PI_4B":
                return RealGyro(identity, network)
            case _:
                return FakeGyro(identity, network)
