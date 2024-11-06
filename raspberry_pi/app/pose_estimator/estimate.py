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
import app.pose_estimator.factors.apriltag_smooth_batch as apriltag_smooth_batch
import app.pose_estimator.factors.gyro as gyro
import app.pose_estimator.factors.odometry as odometry
from app.pose_estimator.drive_util import DriveUtil
from app.pose_estimator.swerve_drive_kinematics import SwerveDriveKinematics100
from app.pose_estimator.swerve_module_delta import SwerveModuleDeltas
from app.pose_estimator.swerve_module_position import (OptionalRotation2d,
                                                       SwerveModulePosition100,
                                                       SwerveModulePositions)

# TODO: real noise estimates.
ODOMETRY_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
# prior uncertainty is larger than field, i.e. "no idea"
PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([16, 8, 6]))
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
        self.isam: gtsam.BatchFixedLagSmoother = self.make_smoother()
        self.result: gtsam.Values = gtsam.Values()
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
        self.positions = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )
        # remember the previous odo input in order to do the delta
        self.odo_t = None
        # for when we make a state but don't have any odometry for it
        self.default_prior = gtsam.Pose2(0, 0, 0)
        self.default_prior_noise = noiseModel.Diagonal.Sigmas(np.array([10, 10, 10]))
        # most-recent odometry
        # remember this so we can use it for extrapolation.
        # TODO: keep track of how old it is?
        self.measurement = Twist2d()
        # duration of most-recent odometry
        self.odo_dt = 0

    def init(self) -> None:
        """Adds camera cal (K) and offset (C) at t0.
        No longer adds a pose at t0, if you want that, do it."""
        # there is just one camera factor
        self.new_values.insert(K(0), CAL)
        self.new_timestamps[K(0)] = 0
        self.new_factors.push_back(gtsam.PriorFactorCal3DS2(K(0), CAL, CAL_NOISE))
        # and one camera offset for now
        self.new_values.insert(C(0), OFFSET0)
        self.new_timestamps[C(0)] = 0
        self.new_factors.push_back(gtsam.PriorFactorPose3(C(0), OFFSET0, OFFSET_NOISE))

    def add_state(self, time_us: int, initial_value: gtsam.Pose2) -> None:
        """Add a new robot state (pose) to the estimator, if it doesn't already exist."""
        # print("add state " , time_us)
        if self.result.exists(X(time_us)):
            # print("it's already in the model")
            return
        if self.new_values.exists(X(time_us)):
            # print("it's already in the values")
            return
        # if you're using the batch smoother, the initial value almost doesn't matter:
        # TODO: use the previous pose as the initial value
        self.new_values.insert(X(time_us), initial_value)
        self.new_timestamps[X(time_us)] = time_us

    def prior(
        self,
        time_us: int,
        value: gtsam.Pose2,
        noise: SharedNoiseModel,
    ) -> None:
        """Prior can have wide noise model (when we really don't know)
        or narrow (for resetting) or mixed (to reset rotation alone)"""
        self.new_factors.push_back(
            gtsam.PriorFactorPose2(
                X(time_us),
                value,
                noise,
            )
        )
        # use this prior if there's a hole to fill
        self.default_prior = value
        self.default_prior_noise = noise

    def odometry(
        self,
        t1_us: int,
        positions: SwerveModulePositions,
        noise: SharedNoiseModel,
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

        # print("odo time ", t1_us)
        if self.odo_t is None:
            # no previous state to refer to.
            # if this happens then the current state will likely
            # have no factors, so add a prior
            self.prior(t1_us, self.default_prior, self.default_prior_noise)
            # print("odo_t null")
            self.positions = positions
            self.odo_t = t1_us
            return
        
        t0_us = self.odo_t

        deltas: SwerveModuleDeltas = DriveUtil.module_position_delta(
            self.positions, positions
        )
        # this is the tangent-space (twist) measurement
        self.measurement: Twist2d = self.kinematics.to_twist_2d(deltas)
        self.odo_dt = t1_us - t0_us
        # print("add odometry factor ", t0_us, t1_us, self.measurement)
        self.new_factors.push_back(
            odometry.factor(
                self.measurement,
                noise,
                X(t0_us),
                X(t1_us),
            )
        )

        self.positions = positions
        self.odo_t = t1_us

    def odometry_custom(
        self, t1_us: int, positions: SwerveModulePositions
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

        if self.odo_t is None:
            # no previous state to refer to
            self.positions = positions
            self.odo_t = t1_us
            return
        
        t0_us = self.odo_t

        deltas: SwerveModuleDeltas = DriveUtil.module_position_delta(
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
        self.odo_t = t1_us

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

    def gyro(self, t0_us: int, t1_us: int, dtheta: float) -> None:
        self.new_factors.push_back(
            gyro.factor(
                np.array([dtheta]),
                GYRO_NOISE,
                X(t0_us),
                X(t1_us),
            )
        )

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

    def apriltag_for_smoothing_batch(
        self,
        landmarks: list[np.ndarray],
        measured: np.ndarray,
        t0_us: int,
        camera_offset: gtsam.Pose3,
        calib: gtsam.Cal3DS2,
    ) -> None:
        """landmarks: list of 3d points
        measured: concatenated px measurements
        TODO: flatten landmarks"""
        noise = noiseModel.Diagonal.Sigmas(np.concatenate([[1, 1] for _ in landmarks]))
        self.new_factors.push_back(
            apriltag_smooth_batch.factor(
                landmarks, measured, camera_offset, calib, noise, X(t0_us)
            )
        )

    def update(self) -> None:
        """Run the solver"""
        self.isam.update(self.new_factors, self.new_values, self.new_timestamps)
        self.result: gtsam.Values = self.isam.calculateEstimate()  # type: ignore

        # print("TIMESTAMPS")
        # print(self.isam.timestamps())
        k = max(self.isam.timestamps().keys())
        ts = max(self.isam.timestamps().values())
        print(self.result.atPose2(k))
        # print(ts)

        # reset the accumulators
        self.new_factors.resize(0)
        self.new_values.clear()
        self.new_timestamps.clear()

    def get_result(self) -> tuple[int, gtsam.Pose2, np.ndarray] | None:
        """the most recent timestamped pose and covariance
        tuple(time_us, pose2, cov)
        TODO: maybe make update() do this"""
        timestamp_map = self.isam.timestamps()
        factors = self.isam.getFactors()
        m = gtsam.Marginals(factors, self.result)
        # timestamp map is std::map inside, which is ordered by key
        for key, time_us in reversed(list(timestamp_map.items())):
            # run through the list from newest to oldest, looking for X
            idx = gtsam.symbolIndex(key)
            char = chr(gtsam.symbolChr(key))
            # print("KEY", key, "IDX", idx, "CHR", char)
            if char == "x":
                # the most-recent pose
                x: gtsam.Pose2 = self.result.atPose2(key)
                cov: np.ndarray = m.marginalCovariance(key)
                return (int(time_us), x, cov)

        return None

    def marginals(self) -> np.ndarray:
        """marginal covariance of most-recent pose
        this is just for testing"""
        timestamp_map = self.isam.timestamps()
        for key, _ in reversed(list(timestamp_map.items())):
            # run through the list from newest to oldest, looking for X
            char = chr(gtsam.symbolChr(key))
            if char == "x":
                factors = self.isam.getFactors()
                m = gtsam.Marginals(factors, self.result)
                return m.marginalCovariance(key)
        return np.array([])

    def joint_marginals(self) -> gtsam.JointMarginal:
        """Joint marginals of the two most recent pose estimates,
        which results in a 6x6 matrix.

        I thought this would be useful for extrapolation, but
        it's not.  Maybe delete it?"""
        timestamp_map = self.isam.timestamps()
        keys = []
        for key, _ in reversed(list(timestamp_map.items())):
            # run through the list from newest to oldest, looking for X
            char = chr(gtsam.symbolChr(key))
            if char == "x":
                keys.append(key)
            if len(keys) > 1:
                break
        factors = self.isam.getFactors()
        m = gtsam.Marginals(factors, self.result)
        return m.jointMarginalCovariance(keys)

    def extrapolate(self, pose: gtsam.Pose2) -> gtsam.Pose2:
        """We're not going to actually use this, we'll let the roboRIO do it."""
        # we should be using Pose2.expmap but it's not wrapped
        # TODO: add it
        p: Pose2d = Pose2d(pose.x(), pose.y(), Rotation2d(pose.theta()))
        p1 = p.exp(self.measurement)
        return gtsam.Pose2(p1.X(), p1.Y(), p1.rotation().radians())

    def make_smoother(self) -> gtsam.BatchFixedLagSmoother:
        # experimenting with the size of the lag buffer.
        # the python odometry factor is intolerably slow
        # but the native one is quite fast.
        # i'm not sure what sort of window we really need; maybe
        # just long enough to span periods of blindness, so,
        # like a second or two?
        # a long window is VERY SLOW, so try very short windows
        # just long enough to catch a single vision update.
        lag_s = 0.1
        lag_us = lag_s * 1e6
        lm_params = gtsam.LevenbergMarquardtParams()
        return gtsam.BatchFixedLagSmoother(lag_us, lm_params)
