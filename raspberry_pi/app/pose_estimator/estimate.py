""" Run a GTSAM model that uses constant camera calibration.

The general idea is to run a free-running loop, polling for inputs.
Those inputs are asserted using one of the input methods here.
"""

# pylint: disable=C0301,E0611,E1101,R0402,R0902,R0903,R0913

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.noiseModel import Base as SharedNoiseModel  # type:ignore
from gtsam.symbol_shorthand import C, K, X  # type:ignore
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Twist2d

import app.pose_estimator.factors.accelerometer as accelerometer
import app.pose_estimator.factors.apriltag_calibrate as apriltag_calibrate
import app.pose_estimator.factors.apriltag_smooth as apriltag_smooth
import app.pose_estimator.factors.gyro as gyro
import app.pose_estimator.factors.odometry as odometry
from app.pose_estimator.drive_util import DriveUtil
from app.pose_estimator.swerve_drive_kinematics import SwerveDriveKinematics100
from app.pose_estimator.swerve_module_delta import SwerveModuleDelta
from app.pose_estimator.swerve_module_position import (OptionalRotation2d,
                                                       SwerveModulePosition100)

# TODO: real noise estimates.
ODOMETRY_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
ACCELEROMETER_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))
GYRO_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.05]))

PX_NOISE = noiseModel.Diagonal.Sigmas(np.array([1, 1]))
# used for calibration only
CAL = gtsam.Cal3DS2(60.0, 60.0, 0.0, 45.0, 45.0, 0.0, 0.0, 0.0, 0.0)
CAL_NOISE = noiseModel.Diagonal.Sigmas(
    np.array(
        [
            1,
            1,
            1,
            1,
            1,
            0.01,
            0.01,
            0.001,
            0.001,
        ]
    )
)
OFFSET0 = gtsam.Pose3()
OFFSET_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))


class Estimate:
    def __init__(self) -> None:
        """Initialize the model
        initial module positions are at their origins.
        TODO: some other initial positions?"""
        self.isam: gtsam.FixedLagSmoother = self.make_smoother()
        self.result: gtsam.Values
        # between updates we accumulate inputs here
        self.new_factors = gtsam.NonlinearFactorGraph()
        self.new_values = gtsam.Values()
        self.new_timestamps = {}
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

        # to keep track of theta increments
        self.theta: float = 0
        # TODO: maybe keeping these states here is a bad idea

    def init(self, initial_pose: Pose2d) -> None:
        """Add a state at zero"""
        prior_mean = gtsam.Pose2(
            initial_pose.X(), initial_pose.Y(), initial_pose.rotation().radians()
        )
        self.add_state(0, prior_mean)
        self.prior(0, prior_mean, PRIOR_NOISE)
        # there is just one camera factor
        self.new_values.insert(K(0), CAL)
        self.new_timestamps[K(0)] = 0
        self.new_factors.push_back(gtsam.PriorFactorCal3DS2(K(0), CAL, CAL_NOISE))
        # and one camera offset for now
        self.new_values.insert(C(0), OFFSET0)
        self.new_timestamps[C(0)] = 0
        self.new_factors.push_back(gtsam.PriorFactorPose3(C(0), OFFSET0, OFFSET_NOISE))

    def add_state(self, time_us: int, initial_value: gtsam.Pose2) -> None:
        """Add a new robot state (pose) to the estimator."""
        # if you're using the batch smoother, the initial value almost doesn't matter:
        # TODO: use the previous pose as the initial value
        self.new_values.insert(X(time_us), initial_value)
        self.new_timestamps[X(time_us)] = time_us

    def prior(self, time_us: int, value: gtsam.Pose2, noise: SharedNoiseModel) -> None:
        """Prior can have wide noise model (when we really don't know)
        or narrow (for resetting) or mixed (to reset rotation alone)"""
        self.new_factors.push_back(
            gtsam.PriorFactorPose2(
                X(time_us),
                value,
                noise,
            )
        )

    def odometry(
        self, t0_us: int, t1_us: int, positions: list[SwerveModulePosition100]
    ) -> None:
        """Add an odometry measurement.  Remember to call add_state so that
        the odometry factor has something to refer to.

        t0_us, t1_us: network tables timestamp in integer microseconds.
        TODO: something more clever with timestamps
        """
        # odometry is not guaranteed to arrive in order, but for now, it is.
        # TODO: out-of-order odometry
        # each odometry update maps exactly to a "between" factor
        # remember a "twist" is a robot-relative concept
        deltas: list[SwerveModuleDelta] = DriveUtil.module_position_delta(
            self.positions, positions
        )
        # this is the tangent-space (twist) measurement
        measurement: Twist2d = self.kinematics.to_twist_2d(deltas)

        self.new_factors.push_back(
            odometry.factor(
                measurement,
                ODOMETRY_NOISE,
                X(t0_us),
                X(t1_us),
            )
        )

        self.positions = positions

    def odometry_custom(
        self, t0_us: int, t1_us: int, positions: list[SwerveModulePosition100]
    ) -> None:
        """Add an odometry measurement.  Remember to call add_state so that
        the odometry factor has something to refer to.

        t0_us, t1_us: network tables timestamp in integer microseconds.
        TODO: something more clever with timestamps
        """
        # odometry is not guaranteed to arrive in order, but for now, it is.
        # TODO: out-of-order odometry
        # each odometry update maps exactly to a "between" factor
        # remember a "twist" is a robot-relative concept
        deltas: list[SwerveModuleDelta] = DriveUtil.module_position_delta(
            self.positions, positions
        )
        # this is the tangent-space (twist) measurement
        measurement: Twist2d = self.kinematics.to_twist_2d(deltas)

        self.new_factors.push_back(
            odometry.factorCustom(
                measurement,
                ODOMETRY_NOISE,
                X(t0_us),
                X(t1_us),
            )
        )

        self.positions = positions

    def accelerometer(
        self, t0_us: int, t1_us: int, t2_us: int, x: float, y: float
    ) -> None:
        """Add an accelerometer measurement.
        t0_us, t1_us, t2_us: timestamps of the referenced states
        TODO: handle time differently
        """
        dt1: float = (t1_us - t0_us) / 1000000
        dt2: float = (t2_us - t1_us) / 1000000
        self.new_factors.push_back(
            accelerometer.factor(
                x,
                y,
                dt1,
                dt2,
                ACCELEROMETER_NOISE,
                X(t0_us),
                X(t1_us),
                X(t2_us),
            )
        )

    def gyro(self, t0_us: int, t1_us: int, theta: float) -> None:
        dtheta = theta - self.theta

        self.new_factors.push_back(
            gyro.factor(
                np.array([dtheta]),
                GYRO_NOISE,
                X(t0_us),
                X(t1_us),
            )
        )
        self.theta = theta

    def apriltag_for_calibration(
        self, landmark: np.ndarray, measured: np.ndarray, t0_us: int
    ) -> None:
        self.new_factors.push_back(
            apriltag_calibrate.factor(
                landmark, measured, PX_NOISE, X(t0_us), C(0), K(0)
            )
        )

    def apriltag_for_smoothing(
        self,
        landmark: np.ndarray,
        measured: np.ndarray,
        t0_us: int,
        camera_offset: gtsam.Pose3,
        calib: gtsam.Cal3DS2,
    ) -> None:
        self.new_factors.push_back(
            apriltag_smooth.factor(
                landmark, measured, camera_offset, calib, PX_NOISE, X(t0_us)
            )
        )

    def update(self) -> None:
        """Run the solver"""
        self.isam.update(self.new_factors, self.new_values, self.new_timestamps)
        self.result = self.isam.calculateEstimate()  # type: ignore
        # reset the accumulators
        self.new_factors.resize(0)
        self.new_values.clear()
        self.new_timestamps.clear()

    def make_smoother(self) -> gtsam.FixedLagSmoother:
        # experimenting with the size of the lag buffer.
        # the python odometry factor is intolerably slow
        # but the native one is quite fast.
        # i'm not sure what sort of window we really need; maybe
        # just long enough to span periods of blindness, so,
        # like a second or two?
        # a long window is VERY SLOW, so try very short windows
        # just long enough to catch a single vision update.
        lag_s = 1
        lag_us = lag_s * 1e6
        lm_params = gtsam.LevenbergMarquardtParams()
        return gtsam.BatchFixedLagSmoother(lag_us, lm_params)
