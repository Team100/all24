# pylint: disable=invalid-name,too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement

import numpy as np
import gtsam  # type:ignore
from gtsam.symbol_shorthand import X, L  # type:ignore
from plot_utils import plot_result, MultivariateNormalParameters  # type:ignore


def extract_pose_marginals(isam, variables, result) -> list:
    pose_marginals = []
    for var in variables:
        pose_marginals.append(
            MultivariateNormalParameters(
                result.atPose2(var), isam.marginalCovariance(var)
            )
        )
    return pose_marginals


def extract_landmark_marginals(isam, variables, result) -> list:
    landmark_marginals = []
    for var in variables:
        landmark_marginals.append(
            MultivariateNormalParameters(
                result.atPoint2(var), isam.marginalCovariance(var)
            )
        )
    return landmark_marginals


def step1_initialize(isam, pose_variables):
    graph = gtsam.NonlinearFactorGraph()
    X1 = X(1)
    pose_variables.append(X1)
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
    graph.add(gtsam.PriorFactorPose2(X1, gtsam.Pose2(0.0, 0.0, 0.0), prior_noise))
    initial_estimate = gtsam.Values()
    initial_estimate.insert(X1, gtsam.Pose2(-0.25, 0.20, 0.15))
    isam.update(graph, initial_estimate)


def step2_add_landmark_observation(isam, landmark_variables):
    graph = gtsam.NonlinearFactorGraph()
    X1 = X(1)
    L1 = L(1)
    landmark_variables.append(L1)
    measurement_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))
    graph.add(
        gtsam.BearingRangeFactor2D(
            X1,
            L1,
            gtsam.Rot2.fromDegrees(45),
            np.sqrt(4.0 + 4.0),
            measurement_noise,
        )
    )
    initial_estimate = gtsam.Values()
    initial_estimate.insert(L1, gtsam.Point2(1.80, 2.10))
    isam.update(graph, initial_estimate)


def step3_add_pose_from_odometry(isam, pose_variables):
    graph = gtsam.NonlinearFactorGraph()
    X1 = X(1)
    X2 = X(2)
    pose_variables.append(X2)
    odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
    graph.add(
        gtsam.BetweenFactorPose2(X1, X2, gtsam.Pose2(2.0, 0.0, 0.0), odometry_noise)
    )
    initial_estimate = gtsam.Values()
    initial_estimate.insert(X2, gtsam.Pose2(2.30, 0.10, -0.20))
    isam.update(graph, initial_estimate)


def step4_add_new_landmark_observation(isam):
    graph = gtsam.NonlinearFactorGraph()
    X2 = X(2)
    L1 = L(1)
    measurement_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))
    graph.add(
        gtsam.BearingRangeFactor2D(
            X2, L1, gtsam.Rot2.fromDegrees(90), 2.0, measurement_noise
        )
    )
    # There are no new variables this update, so no new initial values.
    isam.update(graph, gtsam.Values())


def step5_add_odometry_and_landmark_observations(
    isam, pose_variables, landmark_variables
):
    graph = gtsam.NonlinearFactorGraph()

    X2 = X(2)
    X3 = X(3)
    L2 = L(2)

    pose_variables.append(X3)
    landmark_variables.append(L2)

    odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
    graph.add(
        gtsam.BetweenFactorPose2(X2, X3, gtsam.Pose2(2.0, 0.0, 0.0), odometry_noise)
    )

    measurement_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))
    graph.add(
        gtsam.BearingRangeFactor2D(
            X3, L2, gtsam.Rot2.fromDegrees(90), 2.0, measurement_noise
        )
    )

    # Set initial estimates only for the new variables.
    initial_estimate = gtsam.Values()
    initial_estimate.insert(X3, gtsam.Pose2(4.10, 0.10, 0.10))
    initial_estimate.insert(L2, gtsam.Point2(4.10, 1.80))

    isam.update(graph, initial_estimate)


def main():
    isam = gtsam.ISAM2(gtsam.ISAM2Params())

    pose_variables = []
    landmark_variables = []

    step1_initialize(isam, pose_variables)

    result = isam.calculateEstimate()
    pose_marginals = extract_pose_marginals(isam, pose_variables, result)
    landmark_marginals = extract_landmark_marginals(isam, landmark_variables, result)

    plot_result(pose_marginals, landmark_marginals)

    step2_add_landmark_observation(isam, landmark_variables)

    for _ in range(5):
        isam.update()

    result = isam.calculateEstimate()
    pose_marginals = extract_pose_marginals(isam, pose_variables, result)
    landmark_marginals = extract_landmark_marginals(isam, landmark_variables, result)

    plot_result(pose_marginals, landmark_marginals)

    step3_add_pose_from_odometry(isam, pose_variables)

    for _ in range(5):
        isam.update()

    result = isam.calculateEstimate()
    pose_marginals = extract_pose_marginals(isam, pose_variables, result)
    landmark_marginals = extract_landmark_marginals(isam, landmark_variables, result)

    plot_result(pose_marginals, landmark_marginals)

    step4_add_new_landmark_observation(isam)

    for _ in range(5):
        isam.update()

    result = isam.calculateEstimate()
    pose_marginals = extract_pose_marginals(isam, pose_variables, result)
    landmark_marginals = extract_landmark_marginals(isam, landmark_variables, result)

    plot_result(pose_marginals, landmark_marginals)

    step5_add_odometry_and_landmark_observations(
        isam, pose_variables, landmark_variables
    )

    for _ in range(5):
        isam.update()

    result = isam.calculateEstimate()
    pose_marginals = extract_pose_marginals(isam, pose_variables, result)
    landmark_marginals = extract_landmark_marginals(isam, landmark_variables, result)
    plot_result(pose_marginals, landmark_marginals)


if __name__ == "__main__":
    main()
