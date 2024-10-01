package org.team100.lib.follower;

import java.util.Optional;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.ChassisSpeedsLogger;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.Rotation2dLogger;
import org.team100.lib.logging.SupplierLogger2.Twist2dLogger;
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Stateless (except for loggers) calculations for trajectory followers. */
public class DriveTrajectoryFollowerUtil implements Glassy {
    // LOGGERS
    private final ChassisSpeedsLogger m_log_u_FF;
    private final ChassisSpeedsLogger m_log_u_FB;
    private final ChassisSpeedsLogger m_log_velocity_error;
    private final ChassisSpeedsLogger m_log_u_VFB;
    private final Rotation2dLogger m_log_motion_direction;
    private final DoubleSupplierLogger2 m_log_setpoint_velocity;
    private final Twist2dLogger m_log_position_error;

    public DriveTrajectoryFollowerUtil(SupplierLogger2 parent) {
        SupplierLogger2 child = parent.child(this);
        m_log_u_FF = child.chassisSpeedsLogger(Level.TRACE, "u_FF");
        m_log_u_FB = child.chassisSpeedsLogger(Level.TRACE, "u_FB");
        m_log_velocity_error = child.chassisSpeedsLogger(Level.TRACE, "velocityError");
        m_log_u_VFB = child.chassisSpeedsLogger(Level.TRACE, "u_VFB");
        m_log_motion_direction = child.rotation2dLogger(Level.TRACE, "motion direction");
        m_log_setpoint_velocity = child.doubleLogger(Level.TRACE, "setpoint velocity");
        m_log_position_error = child.twist2dLogger(Level.TRACE, "errorTwist");
    }

    /**
     * Returns robot-relative direction of motion.
     */
    private Optional<Rotation2d> direction(Pose2d measurement, TimedPose setpoint) {
        // Field relative
        Optional<Rotation2d> course = setpoint.state().getCourse();
        if (course.isEmpty())
            return Optional.empty();
        // Adjust course by ACTUAL heading rather than planned to decouple heading and
        // translation errors.
        Rotation2d motion_direction = measurement.getRotation().unaryMinus().rotateBy(course.get());
        m_log_motion_direction.log(() -> motion_direction);
        return Optional.of(motion_direction);
    }

    /**
     * Returns robot-relative speeds
     */
    public ChassisSpeeds feedforward(Pose2d currentPose, TimedPose setpoint) {
        final double velocity_m = setpoint.velocityM_S();
        m_log_setpoint_velocity.log(() -> velocity_m);

        // robot-relative motion direction
        Optional<Rotation2d> motion_direction = direction(currentPose, setpoint);

        ChassisSpeeds u_FF = ff(setpoint, velocity_m, motion_direction);
        m_log_u_FF.log(() -> u_FF);
        return u_FF;
    }

    private ChassisSpeeds ff(TimedPose setpoint, double velocity_m, Optional<Rotation2d> rot) {
        // heading rate is rad/m of movement, so multiply by m/s to get rad/s
        double omega = velocity_m * setpoint.state().getHeadingRate();
        if (rot.isPresent()) {
            Rotation2d motion_direction = rot.get();
            double vx = motion_direction.getCos() * velocity_m;
            double vy = motion_direction.getSin() * velocity_m;
            return new ChassisSpeeds(vx, vy, omega);
        }
        return new ChassisSpeeds(0, 0, omega);
    }

    public ChassisSpeeds feedback(
            Pose2d currentPose,
            TimedPose setpoint,
            double kPCart,
            double kPTheta) {
        final Twist2d positionError = getErrorTwist(currentPose, setpoint);
        m_log_position_error.log(() -> positionError);
        ChassisSpeeds u_FB = new ChassisSpeeds(
                kPCart * positionError.dx,
                kPCart * positionError.dy,
                kPTheta * positionError.dtheta);
        m_log_u_FB.log(() -> u_FB);
        return u_FB;
    }

    public ChassisSpeeds velocityFeedback(
            Pose2d currentPose,
            TimedPose setpoint,
            ChassisSpeeds currentRobotRelativeVelocity,
            final double kPCartV,
            final double kPThetaV) {
        final ChassisSpeeds velocityError = getVelocityError(
                currentPose,
                setpoint,
                currentRobotRelativeVelocity);
        m_log_velocity_error.log(() -> velocityError);
        final ChassisSpeeds u_VFB = new ChassisSpeeds(
                kPCartV * velocityError.vxMetersPerSecond,
                kPCartV * velocityError.vyMetersPerSecond,
                kPThetaV * velocityError.omegaRadiansPerSecond);
        m_log_u_VFB.log(() -> u_VFB);
        return u_VFB;
    }

    /**
     * 
     * @param currentPose                  field relative
     * @param setpoint                     field relative
     * @param currentRobotRelativeVelocity *ROBOT RELATIVE*
     * @param kPCartV
     * @param kPThetaV
     * @return
     */
    public ChassisSpeeds fullFeedback(
            Pose2d currentPose,
            TimedPose setpoint,
            final double kPCart,
            final double kPTheta,
            ChassisSpeeds currentRobotRelativeVelocity,
            final double kPCartV,
            final double kPThetaV) {

        // POSITION
        final ChassisSpeeds u_XFB = feedback(
                currentPose,
                setpoint,
                kPCart,
                kPTheta);

        // VELOCITY
        final ChassisSpeeds u_VFB = velocityFeedback(
                currentPose,
                setpoint,
                currentRobotRelativeVelocity,
                kPCartV,
                kPThetaV);

        return u_XFB.plus(u_VFB);
    }

    ChassisSpeeds getVelocityError(
            Pose2d currentPose,
            TimedPose setpoint,
            ChassisSpeeds currentRobotRelativeVelocity) {
        final double velocity_m = setpoint.velocityM_S();
        // heading rate is rad/m of movement, so multiply by m/s to get rad/s
        double omega = velocity_m * setpoint.state().getHeadingRate();

        Optional<Rotation2d> rot = direction(currentPose, setpoint);

        if (rot.isPresent()) {
            Rotation2d robotRelativeMotionDirection = rot.get();
            double vx = robotRelativeMotionDirection.getCos() * velocity_m;
            double vy = robotRelativeMotionDirection.getSin() * velocity_m;
            return new ChassisSpeeds(
                    vx - currentRobotRelativeVelocity.vxMetersPerSecond,
                    vy - currentRobotRelativeVelocity.vyMetersPerSecond,
                    omega - currentRobotRelativeVelocity.omegaRadiansPerSecond);
        }
        return new ChassisSpeeds(
                -1.0 * currentRobotRelativeVelocity.vxMetersPerSecond,
                -1.0 * currentRobotRelativeVelocity.vyMetersPerSecond,
                omega - currentRobotRelativeVelocity.omegaRadiansPerSecond);
    }

    /**
     * Returns robot-relative twist representing the position error
     */
    public static Twist2d getErrorTwist(Pose2d measurement, TimedPose setpoint) {
        return measurement.log(setpoint.state().getPose());
    }
}
