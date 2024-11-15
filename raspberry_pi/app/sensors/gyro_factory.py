""" Chose a gyro implementation depending on platform. """

# pylint: disable=0903

from adafruit_platformdetect import Detector  # type:ignore

from app.config.identity import Identity
from app.network.network import Network
from app.sensors.fake_gyro import FakeGyro
from app.sensors.gyro_protocol import Gyro
from app.sensors.real_gyro import RealGyro

# pylint: disable=R0903


class GyroFactory:
    @staticmethod
    def get(identity: Identity, network: Network) -> Gyro:
        detector = Detector()
        match detector.board.id:
            case "GENERIC_LINUX_PC":
                return FakeGyro(identity, network)
            case "RASPBERRY_PI_4B":
                try:
                    return RealGyro(identity, network)
                except ValueError:
                    # tried to find a real gyro, failed.
                    print("Failed to find a real gyro, using a fake one")
                    # TODO: return something that does no work at all
                    return FakeGyro(identity, network)
            case _:
                return FakeGyro(identity, network)
