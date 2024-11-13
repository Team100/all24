package org.team100.lib.localization;

import org.team100.lib.motion.drivetrain.SwerveModel;

import edu.wpi.first.math.geometry.Pose2d;


public interface PoseEstimator100 {

    void put(
            double timestampS,
            Pose2d measurement,
            double[] stateSigma,
            double[] visionSigma);


    SwerveModel get(double timestampS);
}
