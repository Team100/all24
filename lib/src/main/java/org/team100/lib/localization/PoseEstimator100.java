package org.team100.lib.localization;

import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * This interface exists in order to make fakes for testing, and to indicate the
 * part of the pose estimator used by the vision updater.
 */
public interface PoseEstimator100 {
    /**
     * Put a new state estimate based on the supplied pose. If not current,
     * subsequent wheel updates are replayed.
     */
    void put(
            double timestampS,
            Pose2d measurement,
            double[] stateSigma,
            double[] visionSigma);

    /**
     * Sample the state estimate buffer.
     */
    SwerveState get(double timestampS);
}
