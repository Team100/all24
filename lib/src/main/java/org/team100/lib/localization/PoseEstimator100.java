package org.team100.lib.localization;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface PoseEstimator100 {
    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     *
     * This method can be called as infrequently as you want, as long as you are
     * calling update() periodically.
     *
     * @param visionRobotPoseMeters pose as measured by the camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds, same epoch as updateWithTime().
     */
    void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds);

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to
     * change trust in vision measurements after the autonomous period, or to change
     * trust as distance to a vision target increases.
     * 
     * TODO: get rid of this as a public method, adjust the stdevs on every update.
     *
     * @param stateStdDevs             standard deviations of the state. Increase
     *                                 these numbers to trust the state less, i.e.
     *                                 allow it to change faster on update.
     * @param visionMeasurementStdDevs Standard deviations of the vision
     *                                 measurements. Increase these numbers to trust
     *                                 global measurements from vision less. This
     *                                 matrix is in the form [x, y, theta]ᵀ, with
     *                                 units in meters and radians.
     */
    default void setStdDevs(
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {
    }

    /**
     * This is for vision calculations, so that we use the high-accuracy gyro
     * measurement for the correct time in the past.
     */
    Optional<Rotation2d> getSampledRotation(
            double timestampSeconds);
}
