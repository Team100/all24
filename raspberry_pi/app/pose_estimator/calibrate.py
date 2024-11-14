""" Run a GTSAM model that calibrates the camera.

It's separate from the "estimate" model just to keep things simple.

TODO: dedupe some of this stuff
"""

# pylint: disable=C0103,E0611,E1101,R0902

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.noiseModel import Base as SharedNoiseModel  # type:ignore
from gtsam.symbol_shorthand import C, K, X  # type:ignore
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Twist2d

import app.pose_estimator.factors.apriltag_calibrate_batch as apriltag_calibrate_batch
import app.pose_estimator.factors.apriltag_calibrate as apriltag_calibrate
import app.pose_estimator.factors.gyro as gyro
import app.pose_estimator.factors.odometry as odometry
from app.kinodynamics.swerve_drive_kinematics import SwerveDriveKinematics100
from app.kinodynamics.swerve_module_delta import SwerveModuleDeltas
from app.kinodynamics.swerve_module_position import (OptionalRotation2d,
                                                     SwerveModulePosition100,
                                                     SwerveModulePositions)
from app.pose_estimator.util import make_smoother
from app.util.drive_util import DriveUtil

# initial value and prior for camera calibration
CAL = gtsam.Cal3DS2(60.0, 60.0, 0.0, 45.0, 45.0, 0.0, 0.0, 0.0, 0.0)
# calibration prior sigma. TODO: make this much wider.
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
# initial value and prior for camera offset.
OFFSET0 = gtsam.Pose3()
# offset prior sigma.  TODO: make this much wider.
OFFSET_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))

# TODO: real noise estimates.
ODOMETRY_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))

# prior uncertainty is *much* larger than field, i.e. "no idea"
PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([160, 80, 60]))
PRIOR_MEAN = gtsam.Pose2(8, 4, 0)

# the gyro has really low noise.
GYRO_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.0001]))

# sensor noise, +/- one pixel.
# TODO: if you want to model *blur* then you need more noise here, and maybe more in x than y.
PX_NOISE = noiseModel.Diagonal.Sigmas(np.array([1, 1]))


class Calibrate:
    def __init__(self) -> None:
        self._isam: gtsam.BatchFixedLagSmoother = make_smoother()
        self._result: gtsam.Values = gtsam.Values()

        # between updates we accumulate inputs here
        self._new_factors = gtsam.NonlinearFactorGraph()
        self._new_values = gtsam.Values()
        self._new_timestamps = {}

        # TODO: correct wheelbase etc
        self._kinematics = SwerveDriveKinematics100(
            [
                Translation2d(0.5, 0.5),
                Translation2d(0.5, -0.5),
                Translation2d(-0.5, 0.5),
                Translation2d(-0.5, -0.5),
            ]
        )
        # TODO: reset position
        self._positions = SwerveModulePositions(
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
            SwerveModulePosition100(0, OptionalRotation2d(True, Rotation2d(0))),
        )
        # remember the previous odo input in order to do the delta
        self._odo_t = None

        # for when we make a state but don't have any odometry for it
        self._default_prior = gtsam.Pose2(0, 0, 0)
        self._default_prior_noise = noiseModel.Diagonal.Sigmas(np.array([10, 10, 10]))
        # most-recent odometry
        # remember this so we can use it for extrapolation.
        # TODO: keep track of how old it is?
        self.measurement = Twist2d()
        # duration of most-recent odometry
        self.odo_dt = 0

    def init(self) -> None:
        """Adds camera cal (K) and offset (C) at t0."""
        # there is just one camera factor
        self._new_values.insert(K(0), CAL)
        self._new_timestamps[K(0)] = 0
        self._new_factors.push_back(gtsam.PriorFactorCal3DS2(K(0), CAL, CAL_NOISE))
        # and one camera offset for now
        self._new_values.insert(C(0), OFFSET0)
        self._new_timestamps[C(0)] = 0
        self._new_factors.push_back(gtsam.PriorFactorPose3(C(0), OFFSET0, OFFSET_NOISE))

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

    def sigma(self, key) -> np.ndarray:
        m = self.marginal_covariance()
        s = m.marginalCovariance(key)
        return np.sqrt(np.diag(s))

    def mean_pose3(self, key) -> gtsam.Pose3:
        return self._result.atPose3(key)

    def mean_cal3DS2(self, key) -> gtsam.Cal3DS2:
        return self._result.atCal3DS2(key)

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

    def apriltag_for_calibration(
        self, landmark: np.ndarray, measured: np.ndarray, t0_us: int
    ) -> None:
        self._new_factors.push_back(
            apriltag_calibrate.factor(
                landmark, measured, PX_NOISE, X(t0_us), C(0), K(0)
            )
        )

    def apriltag_for_calibration_batch(
        self, landmarks: list[np.ndarray], measured: np.ndarray, t0_us: int
    ) -> None:
        noise = noiseModel.Diagonal.Sigmas(np.concatenate([[1, 1] for _ in landmarks]))
        self._new_factors.push_back(
            apriltag_calibrate_batch.factor(
                landmarks, measured, noise, X(t0_us), C(0), K(0)
            )
        )
        # you need to tell the smoother to hang on to these factors.
        self._new_timestamps[C(0)] = t0_us
        self._new_timestamps[K(0)] = t0_us

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

    def gyro(self, t0_us: int, yaw: float) -> None:
        # if this is the only factor attached to this variable
        # then it will be underconstrained (i.e. no constraint on x or y), which could happen.
        self._new_factors.push_back(gyro.factor(np.array([yaw]), GYRO_NOISE, X(t0_us)))
        # if you have only the gyro (which only constrains yaw)
        # you will fail, so add an extremely loose prior.
        self.prior(t0_us, PRIOR_MEAN, PRIOR_NOISE)
