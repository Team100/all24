"""Estimation stuff I don't need but don't want to delete."""

# pylint: disable=C0301,E0611,E1101,R0402,R0902,R0913

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.noiseModel import Base as SharedNoiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore
from wpimath.geometry import Translation2d, Twist2d

import app.pose_estimator.factors.accelerometer as accelerometer
import app.pose_estimator.factors.apriltag_smooth as apriltag_smooth
import app.pose_estimator.factors.binary_gyro as binary_gyro
import app.pose_estimator.factors.gyro as gyro
import app.pose_estimator.factors.odometry as odometry
from app.util.drive_util import DriveUtil
from app.kinodynamics.swerve_drive_kinematics import SwerveDriveKinematics100
from app.kinodynamics.swerve_module_delta import SwerveModuleDeltas
from app.kinodynamics.swerve_module_position import SwerveModulePositions
from app.pose_estimator.util import make_smoother

# no idea what this should be.  it's probably higher.  :-)
ACCELEROMETER_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))
# the gyro has really low noise.
GYRO_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.001]))
# sensor noise, +/- one pixel.
PX_NOISE = noiseModel.Diagonal.Sigmas(np.array([1, 1]))
# prior uncertainty is *much* larger than field, i.e. "no idea"
PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([160, 80, 60]))
PRIOR_MEAN = gtsam.Pose2(8, 4, 0)
ODOMETRY_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))


class ParkingLot:

    def __init__(self) -> None:
        self._isam: gtsam.BatchFixedLagSmoother = make_smoother(0.1)
        self._result: gtsam.Values = gtsam.Values()
        self._new_factors = gtsam.NonlinearFactorGraph()
        self._new_values = gtsam.Values()
        self._new_timestamps = {}
        self._kinematics = SwerveDriveKinematics100(
            [
                Translation2d(0.5, 0.5),
                Translation2d(0.5, -0.5),
                Translation2d(-0.5, 0.5),
                Translation2d(-0.5, -0.5),
            ]
        )
        # for when we make a state but don't have any odometry for it
        self._default_prior = gtsam.Pose2(0, 0, 0)
        self._default_prior_noise = noiseModel.Diagonal.Sigmas(np.array([10, 10, 10]))
        # remember the previous odo input in order to do the delta
        self._odo_t = None
        # most-recent odometry
        # remember this so we can use it for extrapolation.
        # TODO: keep track of how old it is?
        self.measurement = Twist2d()
        # duration of most-recent odometry
        self.odo_dt = 0

    def init(self) -> None:
        """Adds camera cal (K) and offset (C) at t0.
        No longer adds a pose at t0, if you want that, do it."""
        pass

    def add_state(self, time_us: int, initial_value: gtsam.Pose2) -> None:
        """Add a new robot state (pose) to the estimator, if it doesn't already exist."""
        # print("add state " , time_us)
        if self._result.exists(X(time_us)):
            # print("it's already in the model")
            return
        if self._new_values.exists(X(time_us)):
            # print("it's already in the values")
            return
        # if you're using the batch smoother, the initial value
        # almost doesn't matter:
        # TODO: use the previous pose as the initial value
        self._new_values.insert(X(time_us), initial_value)
        self._new_timestamps[X(time_us)] = time_us

    def update(self) -> None:
        """Run the solver"""
        self._isam.update(self._new_factors, self._new_values, self._new_timestamps)
        self._result: gtsam.Values = self._isam.calculateEstimate()  # type: ignore

        # print("TIMESTAMPS")
        # print(self.isam.timestamps())
        k = max(self._isam.timestamps().keys())
        ts = max(self._isam.timestamps().values())
        # print(self._result.atPose2(k))
        # print(ts)

        # reset the accumulators
        self._new_factors.resize(0)
        self._new_values.clear()
        self._new_timestamps.clear()

    def marginal_covariance(self) -> gtsam.Marginals:
        factors = self._isam.getFactors()
        return gtsam.Marginals(factors, self._result)

    def result_size(self) -> int:
        return self._result.size()

    def mean_pose2(self, key) -> gtsam.Pose2:
        return self._result.atPose2(key)

    def sigma_pose2(self, key) -> np.ndarray:
        m = self.marginal_covariance()
        s = m.marginalCovariance(key)
        return np.sqrt(np.diag(s))

    def prior(
        self,
        time_us: int,
        value: gtsam.Pose2,
        noise: SharedNoiseModel,
    ) -> None:
        """Prior can have wide noise model (when we really don't know)
        or narrow (for resetting) or mixed (to reset rotation alone)"""
        self._new_factors.push_back(
            gtsam.PriorFactorPose2(
                X(time_us),
                value,
                noise,
            )
        )
        # use this prior if there's a hole to fill
        self._default_prior = value
        self._default_prior_noise = noise

    def accelerometer(
        self, t0_us: int, t1_us: int, t2_us: int, x: float, y: float
    ) -> None:
        """Add an accelerometer measurement.
        t0_us, t1_us, t2_us: timestamps of the referenced states
        TODO: handle time differently
        Note I think we shouldn't actually use this factor.
        """
        dt1: float = (t1_us - t0_us) / 1000000
        dt2: float = (t2_us - t1_us) / 1000000
        self._new_factors.push_back(
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

    def binary_gyro(self, t0_us: int, t1_us: int, dtheta: float) -> None:
        """Binary gyro means to calculate gyro deltas and apply the delta
        to two pose states, the same way that odometry works.
        I'd suggest not using this factor."""
        self._new_factors.push_back(
            binary_gyro.factor(
                np.array([dtheta]),
                GYRO_NOISE,
                X(t0_us),
                X(t1_us),
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
        """Uses constant offset and calibration, for a single landmark.
        Since we never have single landmarks, we probably don't
        need this version."""
        self._new_factors.push_back(
            apriltag_smooth.factor(
                landmark, measured, camera_offset, calib, PX_NOISE, X(t0_us)
            )
        )

    def odometry_custom(self, t1_us: int, positions: SwerveModulePositions) -> None:
        """This version uses a custom factor, so it's slower, and we shouldn't use it.

        Add an odometry measurement.  Remember to call add_state so that
        the odometry factor has something to refer to.

        t0_us, t1_us: network tables timestamp in integer microseconds.
        TODO: something more clever with timestamps
        """
        # odometry is not guaranteed to arrive in order, but for now, it is.
        # TODO: out-of-order odometry
        # each odometry update maps exactly to a "between" factor
        # remember a "twist" is a robot-relative concept

        if self._odo_t is None:
            # no previous state to refer to
            self._positions = positions
            self._odo_t = t1_us
            return

        t0_us = self._odo_t

        deltas: SwerveModuleDeltas = DriveUtil.module_position_delta(
            self._positions, positions
        )
        # this is the tangent-space (twist) measurement
        measurement: Twist2d = self._kinematics.to_twist_2d(deltas)
        self._new_factors.push_back(
            odometry.factorCustom(
                measurement,
                ODOMETRY_NOISE,
                X(t0_us),
                X(t1_us),
            )
        )

        self._positions = positions
        self._odo_t = t1_us

    # this is here to make the tests that use it happy.
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
        if self._odo_t is None:
            # no previous state to refer to.
            # if this happens then the current state will likely
            # have no factors, so add a prior
            self.prior(t1_us, self._default_prior, self._default_prior_noise)
            # print("odo_t null")
            self._positions = positions
            self._odo_t = t1_us
            return

        t0_us = self._odo_t

        deltas: SwerveModuleDeltas = DriveUtil.module_position_delta(
            self._positions, positions
        )
        # this is the tangent-space (twist) measurement
        self.measurement: Twist2d = self._kinematics.to_twist_2d(deltas)
        self.odo_dt = t1_us - t0_us
        # print("add odometry factor ", t0_us, t1_us, self.measurement)
        self._new_factors.push_back(
            odometry.factor(
                self.measurement,
                noise,
                X(t0_us),
                X(t1_us),
            )
        )

        self._positions = positions
        self._odo_t = t1_us

    # this is here to make the tests that use it happy.
    def gyro(self, t0_us: int, yaw: float) -> None:
        # if this is the only factor attached to this variable
        # then it will be underconstrained (i.e. no constraint on x or y), which could happen.
        self._new_factors.push_back(gyro.factor(np.array([yaw]), GYRO_NOISE, X(t0_us)))
        # if you have only the gyro (which only constrains yaw)
        # you will fail, so add an extremely loose prior.
        self.prior(t0_us, PRIOR_MEAN, PRIOR_NOISE)
