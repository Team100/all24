""" Run a GTSAM model that uses constant camera calibration.

The general idea is to run a free-running loop, polling for inputs.
Those inputs are asserted using one of the input methods here.
"""

# pylint: disable=C0301,E0611,R0903

import numpy as np
from gtsam import (noiseModel, BatchFixedLagSmoother, Cal3DS2, FixedLagSmoother,
                   CustomFactor, FixedLagSmootherKeyTimestampMap, LevenbergMarquardtParams,
                   KeyVector, NonlinearFactorGraph, Point2, Point3, Pose2, Pose3, Rot3,
                   Values)
from gtsam.symbol_shorthand import X
from wpimath.geometry import Rotation2d, Translation2d

from app.pose_estimator.drive_util import DriveUtil
from app.pose_estimator.swerve_drive_kinematics import SwerveDriveKinematics100
from app.pose_estimator.swerve_module_position import (OptionalRotation2d,
                                                       SwerveModulePosition100)

KeyTimestampMap = dict[int, float]

# odometry noise.  TODO: real noise estimate.
NOISE3 = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))


class Estimate:
    def __init__(self) -> None:
        """Initialize the model"""
        self.isam = self.make_smoother()
        self.result: Values
        # between updates we accumulate inputs here
        self.new_factors = NonlinearFactorGraph()
        self.new_theta = Values()
        self.timestamps = KeyTimestampMap()
        # TODO: correct wheelbase etc
        self.kinematics = SwerveDriveKinematics100(
            [
                Translation2d(0.5, 0.5),
                Translation2d(0.5, -0.5),
                Translation2d(-0.5, 0.5),
                Translation2d(-0.5, -0.5),
            ]
        )
        # TODO: reset position
        self.positions = [
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        ]
        self.timestamp = 0

   
    def odometry(self, time_s: float, positions: list[SwerveModulePosition100]) -> None:
        """Add an odometry measurement
        TODO: make sure the timestamp uses the correct basis.
        """
        # odometry is not guaranteed to arrive in order, but for now, it is.
        # TODO: out-of-order odometry
        # each odometry update maps exactly to a "between" factor
        # remember a "twist" is a robot-relative concept
        deltas = DriveUtil.module_position_delta(self.positions, positions)
        # each odometry update makes a new state
        # if you're using the batch smoother, the initial value almost doesn't matter:
        self.new_theta.insert(X(time_s), Pose2())
        self.timestamps.insert((X(time_s), time_s))
       
        def error_func(
            this: CustomFactor, v: Values, H: list[np.ndarray]
        ) -> np.ndarray:
            gT1 = v.atPose2(this.keys()[0])
            gT2 = v.atPose2(this.keys()[1])
            odo = gT1.between(gT2)
            # TODO
            error = robot_delta.localCoordinates(odo)
            if H is not None:
                H[0] = -odo.inverse().AdjointMap()
                H[1] = np.eye(3)
            return error
       
        self.new_factors.add(
            CustomFactor(
                NOISE3,
                KeyVector([X(self.timestamp), X(time_s)]),
                error_func
            )
        )
        self.positions = positions
        self.timestamp = time_s

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
