"""Factory for accelerometer factors.

The GTSAM version of this idea is kind of buried in the
IMUFactor; this is simpler.
"""

# pylint: disable=C0103,E0611
import gtsam
import numpy as np
from gtsam.noiseModel import Base as SharedNoiseModel

def h(p0: gtsam.Pose2, p1: gtsam.Pose2, p2: gtsam.Pose2) -> np.ndarray:

def h_H(measured: np.ndarray, p0: gtsam.Pose2, p1: gtsam.Pose2, p2: gtsam.Pose2, H: list[np.ndarray]):
    result = h(p0, p1, p2) - measured
    if H is not None:
        H[0] =
        H[1] =
        H[2] =
    return result

def factor(x: float,
           y:float,
           model: SharedNoiseModel,
           p0_key: gtsam.Symbol,
           p1_key: gtsam.Symbol,
           p2_key: gtsam.Symbol) -> gtsam.NonlinearFactor:
