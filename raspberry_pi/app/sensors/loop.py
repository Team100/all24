"""Read sensors, send measurements, repeat"""

# pylint: disable=R0903

import time
from threading import Event

from typing_extensions import override

from app.framework.looper import Looper
from app.sensors.gyro_protocol import Gyro


class GyroLoop(Looper):
    def __init__(self, gyro: Gyro, done: Event) -> None:
        self.gyro = gyro
        self.done = done

    @override
    def run(self) -> None:
        try:
            while True:
                # the gyro only has new data every 10 ms,
                # so don't bother sampling more often than that.
                time.sleep(0.01)
                if self.done.is_set():  # exit cleanly
                    return
                self.gyro.sample()
        finally:
            self.done.set()
