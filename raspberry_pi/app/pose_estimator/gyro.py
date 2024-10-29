"""Factory for gyro factors.
"""
# pylint: disable=C0103,E0611,E1101

import gtsam
import numpy as np
from gtsam.noiseModel import Base as SharedNoiseModel

def factor() -> gtsam.NonlinearFactor: