package org.team100.lib.controller;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This originated in DriveMotionPlanner, which included several
 * controllers.
 */
public class DrivePIDController {
    public static final Telemetry t = Telemetry.get();

    private Pose2d  mError = GeometryUtil.kPose2dIdentity;

    public Pose2d getError() {
        return mError;
    }


    public void reset() {
        mError = GeometryUtil.kPose2dIdentity;
    }

    public ChassisSpeeds updatePIDChassis(final Pose2d current_state, final TimedPose mSetpoint, ChassisSpeeds chassisSpeeds) {

        mError = GeometryUtil.transformBy(GeometryUtil.inverse(current_state), mSetpoint.state().getPose());


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
