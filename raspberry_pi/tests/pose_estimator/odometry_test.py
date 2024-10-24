# pylint: disable=C0103,E0611,E1101

import unittest

import gtsam
import numpy as np
from gtsam import (BetweenFactorDouble, BetweenFactorPose2, CustomFactor,
                   FixedLagSmoother, ISAM2GaussNewtonParams, KeyVector,
                   NonlinearFactor, NonlinearFactorGraph, Point2, Point3,
                   Pose2, Pose3, Rot2, Rot3, Values)
from gtsam.noiseModel import Base as SharedNoiseModel
from gtsam.noiseModel import Diagonal
from gtsam.symbol_shorthand import X  # robot pose
from wpimath.geometry import Twist2d

from app.pose_estimator.odometry import odometry_factor

NOISE3 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))


class OdometryTest(unittest.TestCase):
    def test_simple(self) -> None:
        f = odometry_factor(Twist2d(0, 0, 0), NOISE3, 0, 1)
        print("keys",f.keys())
        v = gtsam.Values()
        v.insert(0, Pose2())
        v.insert(1, Pose2())
        result = f.unwhitenedError(v)
        self.assertEqual(3, len(result))
        self.assertAlmostEqual(0, result[0])
        self.assertAlmostEqual(0, result[1])
        self.assertAlmostEqual(0, result[2])

