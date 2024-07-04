package org.team100.lib.controller;

import java.util.Optional;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveMotionControllerUtil implements Glassy {
    private final String kName = DriveMotionControllerUtil.class.getSimpleName();
    private final Telemetry.Logger m_logger;

    public DriveMotionControllerUtil(Logger parent) {
        m_logger = parent.child(this);
    }

    /**
     * Returns robot-relative direction of motion.
     */
    private Rotation2d direction(Pose2d measurement, TimedPose setpoint) {
        // Field relative
        Optional<Rotation2d> course = setpoint.state().getCourse();
        Rotation2d motion_direction = course.isPresent() ? course.get() : GeometryUtil.kRotationZero;
        // Adjust course by ACTUAL heading rather than planned to decouple heading and
        // translation errors.
        motion_direction = measurement.getRotation().unaryMinus().rotateBy(motion_direction);
        m_logger.log(Level.TRACE, "motion direction", motion_direction);
        return motion_direction;
    }

    /**
     * Returns robot-relative speeds
     */
    public ChassisSpeeds feedforward(Pose2d currentPose, TimedPose setpoint) {
        final double velocity_m = setpoint.velocityM_S();
        m_logger.logDouble(Level.TRACE, "setpoint velocity", () -> velocity_m);

        // robot-relative motion direction
        Rotation2d motion_direction = direction(currentPose, setpoint);

        double vx = motion_direction.getCos() * velocity_m;
        double vy = motion_direction.getSin() * velocity_m;
        // heading rate is rad/m of movement, so multiply by m/s to get rad/s
        double omega = velocity_m * setpoint.state().getHeadingRate();

        ChassisSpeeds u_FF = new ChassisSpeeds(vx, vy, omega);
        m_logger.log(Level.DEBUG, "u_FF", u_FF);
        return u_FF;
    }

    public ChassisSpeeds feedback(
            Pose2d currentPose,
            TimedPose setpoint,
            double kPCart,
            double kPTheta) {
        final Twist2d positionError = getErrorTwist(currentPose, setpoint);
        m_logger.log(Level.DEBUG, "errorTwist", positionError);
        ChassisSpeeds u_FB = new ChassisSpeeds(
                kPCart * positionError.dx,
                kPCart * positionError.dy,
                kPTheta * positionError.dtheta);
        m_logger.log(Level.DEBUG, "u_FB", u_FB);
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
        m_logger.log(Level.TRACE, "velocityError", velocityError);
        final ChassisSpeeds u_VFB = new ChassisSpeeds(
                kPCartV * velocityError.vxMetersPerSecond,
                kPCartV * velocityError.vyMetersPerSecond,
                kPThetaV * velocityError.omegaRadiansPerSecond);
        m_logger.log(Level.TRACE, "u_VFB", u_VFB);
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
        Rotation2d robotRelativeMotionDirection = direction(currentPose, setpoint);
        double vx = robotRelativeMotionDirection.getCos() * velocity_m;
        double vy = robotRelativeMotionDirection.getSin() * velocity_m;
        // heading rate is rad/m of movement, so multiply by m/s to get rad/s
        double omega = velocity_m * setpoint.state().getHeadingRate();
        return new ChassisSpeeds(
                vx - currentRobotRelativeVelocity.vxMetersPerSecond,
                vy - currentRobotRelativeVelocity.vyMetersPerSecond,
                omega - currentRobotRelativeVelocity.omegaRadiansPerSecond);
    }

    /**
     * Returns robot-relative twist representing the position error
     */
    static Twist2d getErrorTwist(Pose2d measurement, TimedPose setpoint) {
        return measurement.log(setpoint.state().getPose());
    }

    @Override
    public String getGlassName() {
        return "DriveMotionControllerUtil";
    }

}
