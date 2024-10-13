"""Read sensors, send measurements, repeat"""

# pylint: disable=R0903

import time
from threading import Event

from typing_extensions import override

from app.framework.looper import Looper
from app.sensors.gyro_protocol import Gyro


class GyroLoop(Looper):
    def __init__(self, gyro: Gyro, done: Event) -> None:
        super().__init__(done)
        self.gyro = gyro

    @override
    def execute(self) -> None:
        self.gyro.sample()
        # the gyro only has new data every 10 ms,
        # so don't bother sampling more often than that.
        time.sleep(0.01)

    @override
    def end(self) -> None:
        pass
