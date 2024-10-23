""" Run a GTSAM model that uses constant camera calibration.

The general idea is to run a free-running loop, polling for inputs.
Those inputs are asserted using one of the input methods here.
"""

# pylint: disable=C0301,E0611,R0903


import numpy as np
from gtsam import (
    BatchFixedLagSmoother,
    FixedLagSmootherKeyTimestampMap,
    Cal3DS2,
    FixedLagSmoother,
    LevenbergMarquardtParams,
    NonlinearFactorGraph,
    Point2,
    Point3,
    Pose2,
    Pose3,
    Rot3,
    Values,
)

from app.pose_estimator.swerve_module_position import SwerveModulePosition100

KeyTimestampMap = dict[int, float]


class Estimate:
    def __init__(self) -> None:
        """Initialize the model"""
        self.isam = self.make_smoother()
        self.result: Values
        # between updates we accumulate inputs here
        self.new_factors = NonlinearFactorGraph()
        self.new_theta = Values()
        self.timestamps = KeyTimestampMap()

    def odometry(self, positions: list[SwerveModulePosition100]) -> None:
        """Add an odometry measurement"""
        # odometry is not guaranteed to arrive in order, but for now, it is.
        # TODO: out-of-order odometry
        # each odometry update maps exactly to a "between" factor
        # remember a "twist" is a robot-relative concept

    def update(self) -> None:
        """Run the solver"""
        self.result: Values = self.isam.calculateEstimate()  # type: ignore
        # reset the accumulators
        self.new_factors = NonlinearFactorGraph()
        self.new_theta = Values()
        self.timestamps = KeyTimestampMap()

    def make_smoother(self) -> FixedLagSmoother:
        lag_s = 10
        lm_params: LevenbergMarquardtParams = LevenbergMarquardtParams()
        return BatchFixedLagSmoother(lag_s, lm_params)
