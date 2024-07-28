""" This is a wrapper for the gyro.
"""

############################################
#
# Sample code for the LSM6DSOX gyro.
#
# The deployment mode for this hardware would be
# using the I2C of one or more Raspberry Pi's,
# and doing the integration there.
#
# the uncorrected drift of this sensor is high,
# so there will need to be a "calibration" process
# of some kind, e.g. notice when the robot is not
# moving, and measure the offset, or do it once
# at startup, or something.
#

# windows installation
# python3 -m pip install hidapi
# python3 -m pip install adafruit-blinka
# python3 -m pip install adafruit-circuitpython-lsm6ds

# pylint: disable=import-error, too-few-public-methods

import time
import board  # type:ignore
from adafruit_lsm6ds import Rate  # type:ignore
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX  # type:ignore

from app.network import Network

_OFFSET = -0.014935  # for 100hz


class Gyro:
    def __init__(self, network: Network) -> None:
        self.network = network
        self.i2c = board.I2C()
        self.sox = LSM6DSOX(self.i2c)
        # see adafruit_lsm6ds/__init__.py
        self.sox.gyro_data_rate = Rate.RATE_104_HZ
        self.yaw = 0
        self.prev_time = time.time()

    def sample(self) -> None:
        z = self.sox.gyro[2] - _OFFSET
        endtime = time.time()
        duration = endtime - self.prev_time
        self.prev_time = endtime
        dz = z * duration
        self.yaw += dz
        self.network.gyro_yaw.set(self.yaw)
        self.network.gyro_rate.set(z)
