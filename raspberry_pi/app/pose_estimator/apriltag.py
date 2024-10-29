"""Factory for AprilTag detectors.

One type of factor uses constant camera parameters, the other treats
camera parameters as a model variable, for calibration. 
"""

# pylint: disable=C0103,E0611,E1101

import gtsam
import numpy as np
from gtsam.noiseModel import Base as SharedNoiseModel

def factor() -> gtsam.NonlinearFactor: