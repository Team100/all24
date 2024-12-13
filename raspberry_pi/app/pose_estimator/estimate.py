""" Run a GTSAM model that uses constant camera calibration.

The general idea is to run a free-running loop, polling for inputs.
Those inputs are asserted using one of the input methods here.
"""

# pylint: disable=C0301,E0611,E1101,R0402,R0902,R0903,R0913

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.noiseModel import Base as SharedNoiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Twist2d

import app.pose_estimator.factors.apriltag_smooth_batch as apriltag_smooth_batch
import app.pose_estimator.factors.apriltag_smooth as apriltag_smooth
import app.pose_estimator.factors.gyro as gyro
import app.pose_estimator.factors.odometry as odometry
from app.util.drive_util import DriveUtil
from app.kinodynamics.swerve_drive_kinematics import SwerveDriveKinematics100
from app.kinodynamics.swerve_module_delta import SwerveModuleDeltas
from app.kinodynamics.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
    SwerveModulePositions,
)
from app.pose_estimator.util import make_smoother

# TODO: real noise estimates.
ODOMETRY_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))

# prior uncertainty is *much* larger than field, i.e. "no idea"
PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([160, 80, 60]))
PRIOR_MEAN = gtsam.Pose2(8, 4, 0)

# the gyro has really low noise.
# TODO: consolidate with calibrate.py
GYRO_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.001]))

# sensor noise, +/- one pixel.
# TODO: if you want to model *blur* then you need more noise here, and maybe more in x than y.
PX_NOISE = noiseModel.Diagonal.Sigmas(np.array([1, 1]))

# the "batch" mode uses a python CustomFactor that takes multiple landmarks
# and corresponding pixels; the python stuff is very slow.
# the "not batch" mode uses a C++ factor, PlanarProjectionFactor,
# which takes a single landmark at a time (because there's no particular
# performance advantage to batching in C++)
USE_BATCH = False


class Estimate:
    def __init__(self, lag: float) -> None:
        """Initialize the model
        initial module positions are at their origins.
        TODO: some other initial positions?"""
        self._isam: gtsam.BatchFixedLagSmoother = make_smoother(lag)
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

    def apriltag_for_smoothing_batch(
        self,
        landmarks: list[np.ndarray],
        measured: np.ndarray,
        t0_us: int,
        camera_offset: gtsam.Pose3,
        calib: gtsam.Cal3DS2,
    ) -> None:
        """With constant offset and calibration, either uses a python CustomFactor
         to solve a batch at once, or, uses multiple C++ factors.
        landmarks: list of 3d points
        measured: concatenated px measurements
        TODO: flatten landmarks"""
        if USE_BATCH:
            noise = noiseModel.Diagonal.Sigmas(
                np.concatenate([[1, 1] for _ in landmarks])
            )
            self._new_factors.push_back(
                apriltag_smooth_batch.factor(
                    landmarks, measured, camera_offset, calib, noise, X(t0_us)
                )
            )
        else:
            for i, landmark in enumerate(landmarks):
                px = measured[i * 2 : (i + 1) * 2]
                noise = noiseModel.Diagonal.Sigmas(np.array([1, 1]))
                self._new_factors.push_back(
                    apriltag_smooth.factor(
                        landmark, px, camera_offset, calib, noise, X(t0_us)
                    )
                )

    def update(self) -> None:
        """Run the solver"""
        # print("============UPDATE============")
        # print(self._new_factors)
        # print(self._new_values)
        # print(self._new_timestamps)
        self._isam.update(self._new_factors, self._new_values, self._new_timestamps)
        self._result: gtsam.Values = self._isam.calculateEstimate()  # type: ignore

        # print("TIMESTAMPS")
        # print(self.isam.timestamps())
        k = max(self._isam.timestamps().keys())
        ts = max(self._isam.timestamps().values())
        # print(self.result.atPose2(k))
        # print(ts)

        # reset the accumulators
        self._new_factors.resize(0)
        self._new_values.clear()
        self._new_timestamps.clear()

    def get_result(self) -> tuple[int, gtsam.Pose2, np.ndarray] | None:
        """the most recent timestamped pose and covariance
        tuple(time_us, pose2, cov)
        TODO: maybe make update() do this"""
        timestamp_map = self._isam.timestamps()
        m = self.marginal_covariance()
        # timestamp map is std::map inside, which is ordered by key
        for key, time_us in reversed(list(timestamp_map.items())):
            # run through the list from newest to oldest, looking for X
            idx = gtsam.symbolIndex(key)
            char = chr(gtsam.symbolChr(key))
            # print("KEY", key, "IDX", idx, "CHR", char)
            if char == "x":
                # the most-recent pose
                x: gtsam.Pose2 = self._result.atPose2(key)
                cov: np.ndarray = m.marginalCovariance(key)
                return (int(time_us), x, cov)

        return None

    def marginals(self) -> np.ndarray:
        """marginal covariance of most-recent pose
        this is just for testing"""
        timestamp_map = self._isam.timestamps()
        for key, _ in reversed(list(timestamp_map.items())):
            # run through the list from newest to oldest, looking for X
            char = chr(gtsam.symbolChr(key))
            if char == "x":
                m = self.marginal_covariance()
                return m.marginalCovariance(key)
        return np.array([])

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

    def joint_marginals(self) -> gtsam.JointMarginal:
        """Joint marginals of the two most recent pose estimates,
        which results in a 6x6 matrix.

        I thought this would be useful for extrapolation, but
        it's not.  Maybe delete it?"""
        timestamp_map = self._isam.timestamps()
        keys = []
        for key, _ in reversed(list(timestamp_map.items())):
            # run through the list from newest to oldest, looking for X
            char = chr(gtsam.symbolChr(key))
            if char == "x":
                keys.append(key)
            if len(keys) > 1:
                break
        m = self.marginal_covariance()
        return m.jointMarginalCovariance(keys)

    def extrapolate(self, pose: gtsam.Pose2) -> gtsam.Pose2:
        """We're not going to actually use this, we'll let the roboRIO do it."""
        # we should be using Pose2.expmap but it's not wrapped
        # TODO: add it
        p: Pose2d = Pose2d(pose.x(), pose.y(), Rotation2d(pose.theta()))
        p1 = p.exp(self.measurement)
        return gtsam.Pose2(p1.X(), p1.Y(), p1.rotation().radians())
