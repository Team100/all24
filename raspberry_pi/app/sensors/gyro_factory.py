""" Chose a gyro implementation depending on platform. """

# pylint: disable=0903

from adafruit_platformdetect import Detector  # type:ignore
from app.sensors.gyro_protocol import Gyro
from app.sensors.fake_gyro import FakeGyro
from app.localization.network import Network
from app.sensors.real_gyro import RealGyro


class GyroFactory:
    @staticmethod
    def get(network: Network) -> Gyro:
        detector = Detector()
        match detector.board.id:
            case "GENERIC_LINUX_PC":
                return FakeGyro(network)
            case "RASPBERRY_PI_4B":
                return RealGyro(network)
            case _:
                return FakeGyro(network)
