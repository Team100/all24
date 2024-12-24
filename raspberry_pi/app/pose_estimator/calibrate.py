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
from app.kinodynamics.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
    SwerveModulePositions,
)
from app.pose_estimator.util import make_smoother
from app.util.drive_util import DriveUtil

# the initial guesses need to be kinda close
# the prior noise needs to be kinda narrow

# initial value and prior for camera calibration
# in simulation the correct k1 and k2 values appear
# after just 0.25 sec (!)
CAL_PRIOR_MEAN = gtsam.Cal3DS2(
    180.0,  # wrong, the real value is 200
    180.0,  # wrong, the real value is 200
    0.0,
    400.0,  # known precisely
    300.0,  # known precisely
    0.0,
    0.0,
)

# calibration prior sigma.
# some parameters are known very precisely, e.g. s, cx, cy, p1, p2.
# also we can measure the focal length approximately
# distortion coefficients are hard to measure but not very large.
CAL_PRIOR_NOISE = noiseModel.Diagonal.Sigmas(
    np.array(
        [
            10,  # fx
            10,  # fy
            0.001,  # s
            0.001,  # cx
            0.001,  # cy
            1,  # k1
            1,  # k2
            0.001,  # p1
            0.001,  # p2
        ]
    )
)

# initial value and prior for camera offset.
# in simulation, the "z" component is fixed within 0.25 sec (!)
# but the "x" and "y" are *never* fixed, i think because a fixed offset
# just doesn't matter much in the sim geometry.
# remember the camera is now z-fwd
OFFSET_PRIOR_MEAN = gtsam.Pose3(
    gtsam.Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
    np.array(
        [
            0.0,
            0.0,
            0.6,
        ]
    ),
)

# offset prior sigma.
# we can measure the offset within a few inches and tens of degrees
# in gtsam, a pose3 logmap is [R_x,R_y,R_z,T_x,T_y,T_z]
# remember camera is z-fwd
OFFSET_PRIOR_NOISE = noiseModel.Diagonal.Sigmas(
    np.array([0.2, 0.2, 0.1, 0.2, 0.2, 0.2])
)

# TODO: real noise estimates.
ODOMETRY_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))

# prior uncertainty is *much* larger than field, i.e. "no idea"
PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([160, 80, 60]))
PRIOR_MEAN = gtsam.Pose2(8, 4, 0)

# the gyro has really low noise.
# GYRO_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.0001]))
# but if we actually give the model such a tiny error, then it
# freaks out, changing all the other dimensions to try to get an
# answer within those tiny bounds.  so use *slightly* wider
# noise model and it seems to work fine, at least in simulation.
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


class Calibrate:
    def __init__(self, lag_s: float) -> None:
        """lag_s: size of lag window in seconds"""
        self._isam: gtsam.BatchFixedLagSmoother = make_smoother(lag_s)
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
        self._new_values.insert(K(0), CAL_PRIOR_MEAN)
        self._new_timestamps[K(0)] = 0
        self._new_factors.push_back(
            gtsam.PriorFactorCal3DS2(K(0), CAL_PRIOR_MEAN, CAL_PRIOR_NOISE)
        )
        # and one camera offset for now
        self._new_values.insert(C(0), OFFSET_PRIOR_MEAN)
        self._new_timestamps[C(0)] = 0
        self._new_factors.push_back(
            gtsam.PriorFactorPose3(C(0), OFFSET_PRIOR_MEAN, OFFSET_PRIOR_NOISE)
        )

    def update(self) -> None:
        """Run the solver"""
        # print("FACTORS", self._new_factors)
        # print("VALUES", self._new_values)
        # print("TIMESTAMPS", self._new_timestamps)
        self._isam.update(self._new_factors, self._new_values, self._new_timestamps)
        self._result: gtsam.Values = self._isam.calculateEstimate()  # type: ignore

        # print("TIMESTAMPS")
        # print(self._isam.timestamps())
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
        """For a Pose2 this returns [sigma x, sigma y, sigma Θ].
        For a Pose3, [R_x,R_y,R_z,T_x,T_y,T_z] (see Pose3.h)
        For Cal3DS2, [fx, fy, s, cx, cy, k1, k2, p1, p2]
        TODO: make a separate method for each type"""
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
                apriltag_calibrate_batch.factor(
                    landmarks, measured, noise, X(t0_us), C(0), K(0)
                )
            )
        else:
            for i, landmark in enumerate(landmarks):
                px = measured[i * 2 : (i + 1) * 2]
                noise = noiseModel.Diagonal.Sigmas(np.array([1, 1]))
                self._new_factors.push_back(
                    apriltag_calibrate.factor(landmark, px, noise, X(t0_us), C(0), K(0))
                )

    def keep_calib_hot(self, t0_us: int) -> None:
        """Even if we don't see any targets, remember the
        camera calibration and offset."""
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

    def get_result(self) -> tuple[int, gtsam.Pose2, np.ndarray] | None:
        """the most recent timestamped pose and covariance
        tuple(time_us, pose2, cov)
        TODO: maybe make update() do this
        TODO: make more of these, i.e. one for C, K, and X"""
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
