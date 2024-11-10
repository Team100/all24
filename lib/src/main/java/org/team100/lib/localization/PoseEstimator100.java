package org.team100.lib.localization;

import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;


public interface PoseEstimator100 {

    void put(
            double timestampS,
            Pose2d measurement,
            double[] stateSigma,
            double[] visionSigma);


    SwerveState get(double timestampS);
}
