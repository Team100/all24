package org.team100.lib.controller;

import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This originated in DriveMotionPlanner, which included several
 * controllers.
 */
public class DrivePIDController {
    public static final Telemetry t = Telemetry.get();

    public void reset() {

    }

    public ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds, Pose2d mError) {
        // Feedback on longitudinal error (distance).
        final double kPathk = 2.4;
        // 2.4;
        // Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
        // 0.15;
        final double kPathKTheta = 2.4;

        t.log("/drive_pid_controller/error", mError);

        Twist2d pid_error = new Pose2d().log(mError);
        t.log("/drive_pid_controller/pid error", pid_error);

        chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond + kPathk * pid_error.dx;
        chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond + kPathk * pid_error.dy;
        chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond + kPathKTheta * pid_error.dtheta;
        return chassisSpeeds;
    }

}
