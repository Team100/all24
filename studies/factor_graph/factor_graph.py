import numpy as np
import gtsam
from gtsam.symbol_shorthand import X, L
from plot_utils import plot_result, MultivariateNormalParameters

"""Based on the example https://github.com/borglab/gtsam/blob/develop/python/gtsam/examples/PlanarSLAMExample.py"""


def incremental_factorgraph_example():
    # Create an ISAM2 object.
    # You can balance computation time vs accuracy by changing the parameters to ISAM2 in the ISAM2Params struct
    # (see https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/ISAM2Params.h).
    # The parameters are here set to default.
    isam_params = gtsam.ISAM2Params()
    isam = gtsam.ISAM2(isam_params)

    # We will keep the variables in lists for simplicity.
    pose_variables = []
    landmark_variables = []

    # Step 1: Initialize with a prior on the first pose X1.
    def step1_initialize():
        # Create an empty nonlinear factor graph.
        # We will need to do this for every update.
        graph = gtsam.NonlinearFactorGraph()

        # Create a key for the first pose.
        X1 = X(1)

        # Update the list with the new pose variable key.
        pose_variables.append(X1)

        # Add a prior on pose X1 at the origin.
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
        graph.add(gtsam.PriorFactorPose2(X1, gtsam.Pose2(0.0, 0.0, 0.0), prior_noise))

        # Set an initial estimate for the first pose.
        initial_estimate = gtsam.Values()
        initial_estimate.insert(X1, gtsam.Pose2(-0.25, 0.20, 0.15))

        # Update ISAM2 with the initial factor graph.
        isam.update(graph, initial_estimate)

    step1_initialize()

    # Compute the current MAP estimate for all variables.
    result = isam.calculateEstimate()

    # Define a function that extracts the marginals (which we will reuse below).
    # Notice that we compute the marginal covariance directly from the isam object.
    def extract_marginals():
        # Extract the marginal distributions for each variable
        X_marginals = []
        for var in pose_variables:
            X_marginals.append(MultivariateNormalParameters(result.atPose2(var), isam.marginalCovariance(var)))

        L_marginals = []
        for var in landmark_variables:
            L_marginals.append(MultivariateNormalParameters(result.atPoint2(var), isam.marginalCovariance(var)))

        return X_marginals, L_marginals

    pose_marginals, landmark_marginals = extract_marginals()

    # Show current results.
    plot_result(pose_marginals, landmark_marginals)

    # Step 2: Add a landmark observation from the pose X1
    def step2_add_landmark_observation():
        # Create an empty nonlinear factor graph.
        graph = gtsam.NonlinearFactorGraph()

        # Create keys for the involved variables.
        X1 = X(1)
        L1 = L(1)

        # Update the list with the new landmark variable key.
        landmark_variables.append(L1)

        # Add the landmark observation.
        measurement_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))
        graph.add(gtsam.BearingRangeFactor2D(X1, L1, gtsam.Rot2.fromDegrees(45), np.sqrt(4.0 + 4.0), measurement_noise))

        # Set initial estimates only for the new variables.
        initial_estimate = gtsam.Values()
        initial_estimate.insert(L1, gtsam.Point2(1.80, 2.10))

        # Update ISAM2 with the new factor graph.
        isam.update(graph, initial_estimate)

    step2_add_landmark_observation()

    # ISAM2 performs only one iteration of the iterative nonlinear solver per update() call.
    # We can use "free time" until the next update from the sensors to perform additional iterations.
    for _ in range(5):
        isam.update()

    # Compute the updated MAP estimate.
    result = isam.calculateEstimate()
    pose_marginals, landmark_marginals = extract_marginals()

    # Show updated results.
    plot_result(pose_marginals, landmark_marginals)

    # Step 3: Add a new pose with a relative odometry constraint.
    def step3_add_pose_from_odometry():
        # Create an empty nonlinear factor graph.
        graph = gtsam.NonlinearFactorGraph()

        # Create keys for the involved variables.
        X1 = X(1)
        X2 = X(2)

        # Update the list with the new pose variable key.
        pose_variables.append(X2)

        # Add an odometry measurement.
        odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
        graph.add(gtsam.BetweenFactorPose2(X1, X2, gtsam.Pose2(2.0, 0.0, 0.0), odometry_noise))

        # Set initial estimates only for the new variables.
        initial_estimate = gtsam.Values()
        initial_estimate.insert(X2, gtsam.Pose2(2.30, 0.10, -0.20))

        # Update ISAM2 with the new factor graph.
        isam.update(graph, initial_estimate)

    step3_add_pose_from_odometry()

    # Run a few additional update iterations.
    for _ in range(5):
        isam.update()

    # Compute the updated MAP estimate.
    result = isam.calculateEstimate()
    pose_marginals, landmark_marginals = extract_marginals()

    # Show updated results.
    plot_result(pose_marginals, landmark_marginals)

    # Step 4: Add new observation of the landmark L1 from X2.
    def step4_add_new_landmark_observation():
        # Create an empty nonlinear factor graph.
        graph = gtsam.NonlinearFactorGraph()

        # Create keys for the involved variables.
        X2 = X(2)
        L1 = L(1)

        # Add the landmark observation.
        measurement_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))
        graph.add(gtsam.BearingRangeFactor2D(X2, L1, gtsam.Rot2.fromDegrees(90), 2.0, measurement_noise))

        # Update ISAM2 with the new factor graph.
        # There are no new variables this update, so no new initial values.
        isam.update(graph, gtsam.Values())

    step4_add_new_landmark_observation()

    # Run a few additional update iterations.
    for _ in range(5):
        isam.update()

    # Compute the updated MAP estimate.
    result = isam.calculateEstimate()
    pose_marginals, landmark_marginals = extract_marginals()

    # Show updated results.
    plot_result(pose_marginals, landmark_marginals)

    # Step 5: Add both new odometry and landmark observations for X3.
    def step5_add_odometry_and_landmark_observations():
        # Create an empty nonlinear factor graph.
        graph = gtsam.NonlinearFactorGraph()

        # Create keys for the involved variables.
        X2 = X(2)
        X3 = X(3)
        L2 = L(2)

        # Update the list with the new variable keys.
        pose_variables.append(X3)
        landmark_variables.append(L2)

        # Add the odometry measurement.
        odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
        graph.add(gtsam.BetweenFactorPose2(X2, X3, gtsam.Pose2(2.0, 0.0, 0.0), odometry_noise))

        # Add the landmark observation.
        measurement_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))
        graph.add(gtsam.BearingRangeFactor2D(X3, L2, gtsam.Rot2.fromDegrees(90), 2.0, measurement_noise))

        # Set initial estimates only for the new variables.
        initial_estimate = gtsam.Values()
        initial_estimate.insert(X3, gtsam.Pose2(4.10, 0.10, 0.10))
        initial_estimate.insert(L2, gtsam.Point2(4.10, 1.80))

        # Update ISAM2 with the new factor graph.
        isam.update(graph, initial_estimate)

    step5_add_odometry_and_landmark_observations()

    # Run a few additional update iterations.
    for _ in range(5):
        isam.update()

    # Compute the updated MAP estimate.
    result = isam.calculateEstimate()
    pose_marginals, landmark_marginals = extract_marginals()

    # Show updated results.
    plot_result(pose_marginals, landmark_marginals)


if __name__ == "__main__":
    incremental_factorgraph_example()