"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Demonstration of the fixed-lag smoothers using a planar robot example
and multiple odometry-like sensors
Author: Frank Dellaert (C++), Jeremy Aguilon (Python)
"""

import numpy as np

import gtsam
import gtsam_unstable
from gtsam.symbol_shorthand import X, L  # type:ignore


def BatchFixedLagSmootherExample():
    """
    Runs a batch fixed smoother on an agent with two odometry
    sensors that is simply moving to the
    """

    # Define a batch fixed lag smoother, which uses
    # Levenberg-Marquardt to perform the nonlinear optimization
    lag = 0.5
    # smoother_batch = gtsam_unstable.BatchFixedLagSmoother(lag)
    smoother_batch = gtsam_unstable.IncrementalFixedLagSmoother(lag)

    # Create containers to store the factors and linearization points
    # that will be sent to the smoothers
    new_factors = gtsam.NonlinearFactorGraph()
    new_values = gtsam.Values()
    new_timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()

    # Create  a prior on the first pose, placing it at the origin
    prior_mean = gtsam.Pose2(0, 0, 0)
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
    X0 = X(0)
    new_factors.push_back(gtsam.PriorFactorPose2(X0, prior_mean, prior_noise))
    new_values.insert(X0, prior_mean)
    new_timestamps.insert((X0, 0.0))

    smoother_batch.update(new_factors, new_values, new_timestamps)

    # new_timestamps.clear()
    new_timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
    # new_values.clear()
    new_values = gtsam.Values()
    # new_factors.resize(0)
    new_factors = gtsam.NonlinearFactorGraph()


    L1 = L(1)
    L2 = L(2)
    new_factors.add(
        gtsam.PriorFactorPoint2(
            L1,
            np.array([0, 0]),
            gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05])),
        )
    )
    new_factors.add(
        gtsam.PriorFactorPoint2(
            L2,
            np.array([1, 0]),
            gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05])),
        )
    )
    new_values.insert(L1, gtsam.Point2(0, 0))
    new_values.insert(L2, gtsam.Point2(1, 0))
    new_timestamps.insert((L1, 0))
    new_timestamps.insert((L2, 0))
    smoother_batch.update(new_factors, new_values, new_timestamps)
    # new_timestamps.clear()
    new_timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
    # new_values.clear()
    new_values = gtsam.Values()
    # new_factors.resize(0)
    new_factors = gtsam.NonlinearFactorGraph()

    delta_time = 0.25
    time = 0.25

    while time <= 5.0:
        previous_key = int(1000 * (time - delta_time))
        current_key = int(1000 * time)

        # assign current key to the current timestamp
        new_timestamps.insert((X(current_key), time))

        # Add a guess for this pose to the new values
        # Assume that the robot moves at 2 m/s. Position is time[s] * 2[m/s]
        current_pose = gtsam.Pose2(time * 2, 0, 0)
        new_values.insert(X(current_key), current_pose)

        # Add odometry factors from two different sources with different error
        # stats
        odometry_measurement_1 = gtsam.Pose2(0.61, -0.08, 0.02)
        odometry_noise_1 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))
        new_factors.push_back(
            gtsam.BetweenFactorPose2(
                X(previous_key),
                X(current_key),
                odometry_measurement_1,
                odometry_noise_1,
            )
        )

        odometry_measurement_2 = gtsam.Pose2(0.47, 0.03, 0.01)
        odometry_noise_2 = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.05, 0.05, 0.05])
        )
        new_factors.push_back(
            gtsam.BetweenFactorPose2(
                X(previous_key),
                X(current_key),
                odometry_measurement_2,
                odometry_noise_2,
            )
        )

        smoother_batch.update(new_factors, new_values, new_timestamps)

        new_values = gtsam.Values()
        new_factors = gtsam.NonlinearFactorGraph()


        new_factors.add(
            gtsam.BearingRangeFactor2D(
                X(current_key),
                L1,
                gtsam.Rot2.fromAngle(1),
                1,
                gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05])),
            )
        )
        new_factors.add(
            gtsam.BearingRangeFactor2D(
                X(current_key),
                L2,
                gtsam.Rot2.fromAngle(1),
                1,
                gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05])),
            )
        )
        new_timestamps.insert((L1, time))
        new_timestamps.insert((L2, time))

        # Update the smoothers with the new factors. In this case,
        # one iteration must pass for Levenberg-Marquardt to accurately
        # estimate
        if time >= 0.50:
            smoother_batch.update(new_factors, new_values, new_timestamps)
            print("Timestamp = " + str(time) + ", Key = " + str(current_key))
            # print(f"timestamps: {smoother_batch.timestamps().size()}")
            # print(f" key 500? {smoother_batch.calculateEstimate().exists(X(500))}")

            v = smoother_batch.calculateEstimate()
            # print("VALUE: " , v)
            # print("KEYS ", v.keys())
            # print(dir(v))
            # print("VAL " , dir(v.atPose2(X(current_key))))
            # print(smoother_batch.calculateEstimatePose2(X(current_key)))
            # print(smoother_batch.getISAM2().marginalCovariance(X(current_key)))

            # new_timestamps.clear()
            new_timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
            # new_values.clear()
            new_values = gtsam.Values()
            # new_factors.resize(0)
            new_factors = gtsam.NonlinearFactorGraph()


        time += delta_time


if __name__ == "__main__":
    BatchFixedLagSmootherExample()
    print("Example complete")
