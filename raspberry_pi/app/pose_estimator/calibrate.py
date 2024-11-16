""" Run a GTSAM model that calibrates the camera.

It's separate from the "estimate" model just to keep things simple.

TODO: dedupe some of this stuff
"""

# pylint: disable=E0611,E1101

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.noiseModel import Base as SharedNoiseModel  # type:ignore
from gtsam.symbol_shorthand import C, K, X  # type:ignore

import app.pose_estimator.factors.apriltag_calibrate as apriltag_calibrate_batch
from app.pose_estimator.util import make_smoother

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

        # for when we make a state but don't have any odometry for it
        self._default_prior = gtsam.Pose2(0, 0, 0)
        self._default_prior_noise = noiseModel.Diagonal.Sigmas(np.array([10, 10, 10]))

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
        print(self._result.atPose2(k))
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
            apriltag_calibrate_batch.factor(
                landmark, measured, PX_NOISE, X(t0_us), C(0), K(0)
            )
        )
