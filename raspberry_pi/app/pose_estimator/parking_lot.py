"""Stuff I don't need but don't want to delete."""

# pylint: disable=C0301,E0611,E1101,R0913

import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from gtsam.noiseModel import Base as SharedNoiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore

import app.pose_estimator.factors.accelerometer as accelerometer
from app.pose_estimator.util import make_smoother

ACCELEROMETER_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))


class ParkingLot:

    def __init__(self) -> None:
        self.isam: gtsam.BatchFixedLagSmoother = make_smoother()
        self.result: gtsam.Values = gtsam.Values()
        self.new_factors = gtsam.NonlinearFactorGraph()
        self.new_values = gtsam.Values()
        self.new_timestamps = {}

    def init(self) -> None:
        """Adds camera cal (K) and offset (C) at t0.
        No longer adds a pose at t0, if you want that, do it."""
        pass

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
