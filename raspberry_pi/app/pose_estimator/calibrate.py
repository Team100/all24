""" Run a GTSAM model that calibrates the camera.

It's separate from the "estimate" model just to keep things simple.
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
        self.isam: gtsam.BatchFixedLagSmoother = make_smoother()
        self.result: gtsam.Values = gtsam.Values()

        # between updates we accumulate inputs here
        self.new_factors = gtsam.NonlinearFactorGraph()
        self.new_values = gtsam.Values()
        self.new_timestamps = {}

        # for when we make a state but don't have any odometry for it
        self.default_prior = gtsam.Pose2(0, 0, 0)
        self.default_prior_noise = noiseModel.Diagonal.Sigmas(np.array([10, 10, 10]))

    def init(self) -> None:
        """Adds camera cal (K) and offset (C) at t0."""
        # there is just one camera factor
        self.new_values.insert(K(0), CAL)
        self.new_timestamps[K(0)] = 0
        self.new_factors.push_back(gtsam.PriorFactorCal3DS2(K(0), CAL, CAL_NOISE))
        # and one camera offset for now
        self.new_values.insert(C(0), OFFSET0)
        self.new_timestamps[C(0)] = 0
        self.new_factors.push_back(gtsam.PriorFactorPose3(C(0), OFFSET0, OFFSET_NOISE))

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

    def add_state(self, time_us: int, initial_value: gtsam.Pose2) -> None:
        """Add a new robot state (pose) to the estimator, if it doesn't already exist."""
        # print("add state " , time_us)
        if self.result.exists(X(time_us)):
            # print("it's already in the model")
            return
        if self.new_values.exists(X(time_us)):
            # print("it's already in the values")
            return
        # if you're using the batch smoother, the initial value
        # almost doesn't matter:
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

    def apriltag_for_calibration(
        self, landmark: np.ndarray, measured: np.ndarray, t0_us: int
    ) -> None:
        self.new_factors.push_back(
            apriltag_calibrate_batch.factor(
                landmark, measured, PX_NOISE, X(t0_us), C(0), K(0)
            )
        )
