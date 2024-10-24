""" Evaluate the estimation model. """

# pylint: disable=C0301,E0611,R0903

import math
import unittest
from wpimath.geometry import Rotation2d, Translation2d

from gtsam import Cal3DS2, Pose2, Pose3, Point2, Point3, Rot3
import numpy as np

# this works with runtests.py but not the little triangle up there
from app.pose_estimator.estimate import Estimate
from app.pose_estimator.swerve_module_position import OptionalRotation2d, SwerveModulePosition100
from tests.pose_estimator.simulator import Simulator


class EstimateTest(unittest.TestCase):
    def test_eval(self) -> None:
        sim = Simulator()
        # initial position at origin
        est = Estimate()
        # drive straight ahead
        positions = [
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0.1, OptionalRotation2d(True, Rotation2d(0))),
        ]
        time_s = 0.1
        est.odometry(time_s, positions)
        est.update()