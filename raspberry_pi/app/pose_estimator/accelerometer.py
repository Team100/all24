"""Factory for accelerometer factors.

The GTSAM version of this idea is kind of buried in the
IMUFactor; this is simpler.
"""

# pylint: disable=C0103,E0611
import gtsam
from numpy import double
from gtsam.noiseModel import Base as SharedNoiseModel


def factor(x: double,
           y:double,
           model: SharedNoiseModel,
           p0_key: gtsam.Symbol,
           p1_key: gtsam.Symbol,
           p2_key: gtsam.Symbol) -> gtsam.NonlinearFactor:
