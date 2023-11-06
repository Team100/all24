package org.team100.lib.controller;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This originated in DriveMotionPlanner, which included several
 * controllers.
 * 
 * Implements feedforward based on trajectory velocity, and proportional
 * feedback on pose.
 */
public class DrivePIDController implements DriveMotionController {
    public static final Telemetry t = Telemetry.get();

    private TrajectoryTimeIterator mCurrentTrajectory;
    private TimedPose mSetpoint = new TimedPose(new Pose2dWithMotion());
    private Pose2d mError = GeometryUtil.kPose2dIdentity;
    private double mLastTime = Double.POSITIVE_INFINITY;

    @Override
    public void setTrajectory(final TrajectoryTimeIterator trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        mError = GeometryUtil.kPose2dIdentity;
        mLastTime = Double.POSITIVE_INFINITY;
    }

    @Override
    public ChassisSpeeds update(double timestamp, Pose2d current_state, Twist2d current_velocity) {
        return updatePIDChassis(timestamp, current_state);
    }

    public ChassisSpeeds updatePIDChassis(final double timestamp, final Pose2d current_state) {
        if (mCurrentTrajectory == null)
            return null;

        t.log("/pid_planner/current state", current_state);
        if (isDone()) {
            return new ChassisSpeeds();
        }

        if (!Double.isFinite(mLastTime))
            mLastTime = timestamp;
        final double mDt = timestamp - mLastTime;
        mLastTime = timestamp;

        TrajectorySamplePoint sample_point = mCurrentTrajectory.advance(mDt);
        t.log("/pid_planner/sample point", sample_point);
        mSetpoint = sample_point.state();
        t.log("/pid_planner/setpoint", mSetpoint);

        mError = GeometryUtil.transformBy(GeometryUtil.inverse(current_state), mSetpoint.state().getPose());

        final double velocity_m = mSetpoint.velocityM_S();
        t.log("/pid_planner/setpoint velocity", velocity_m);

        // Field relative
        var course = mSetpoint.state().getCourse();
        Rotation2d motion_direction = course.isPresent() ? course.get() : GeometryUtil.kRotationIdentity;
        // Adjust course by ACTUAL heading rather than planned to decouple heading and
        // translation errors.
        motion_direction = current_state.getRotation().unaryMinus().rotateBy(motion_direction);
        t.log("/planner/motion direction", motion_direction);

        // this is feedforward
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                motion_direction.getCos() * velocity_m,
                motion_direction.getSin() * velocity_m,
                // Need unit conversion because Pose2dWithMotion heading rate is per unit
                // distance.
                velocity_m * mSetpoint.state().getHeadingRate());
        t.log("/planner/chassis speeds", chassisSpeeds);

        // Feedback on longitudinal error (distance).
        final double kPathk = 2.4;
        // 2.4;
        // Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
        // 0.15;
        final double kPathKTheta = 2.4;

        t.log("/drive_pid_controller/error", mError);

        Twist2d pid_error = new Pose2d().log(mError);
        t.log("/drive_pid_controller/pid error", pid_error);

        // u = u_FF + K(error)
        chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond + kPathk * pid_error.dx;
        chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond + kPathk * pid_error.dy;
        chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond + kPathKTheta * pid_error.dtheta;
        return chassisSpeeds;
    }

    @Override
    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    // for testing
    TimedPose getSetpoint() {
        return mSetpoint;
    }

    // for testing
    Pose2d getError() {
        return mError;
    }
}
