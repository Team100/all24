package org.team100.lib.controller;

import java.util.Optional;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveMotionControllerUtil {
    private static final Telemetry t = Telemetry.get();
    private static final String kName = DriveMotionControllerUtil.class.getSimpleName();

    /**
     * Returns robot-relative direction of motion.
     */
    private static Rotation2d direction(Pose2d measurement, TimedPose setpoint) {
        // Field relative
        Optional<Rotation2d> course = setpoint.state().getCourse();
        Rotation2d motion_direction = course.isPresent() ? course.get() : GeometryUtil.kRotationZero;
        // Adjust course by ACTUAL heading rather than planned to decouple heading and
        // translation errors.
        motion_direction = measurement.getRotation().unaryMinus().rotateBy(motion_direction);
        t.log(Level.TRACE, kName, "motion direction", motion_direction);
        return motion_direction;
    }

    /**
     * Returns robot-relative speeds
     */
    public static ChassisSpeeds feedforward(Pose2d currentPose, TimedPose setpoint) {
        final double velocity_m = setpoint.velocityM_S();
        t.log(Level.TRACE, kName, "setpoint velocity", velocity_m);

        // robot-relative motion direction
        Rotation2d motion_direction = direction(currentPose, setpoint);

        double vx = motion_direction.getCos() * velocity_m;
        double vy = motion_direction.getSin() * velocity_m;
        // heading rate is rad/m of movement, so multiply by m/s to get rad/s
        double omega = velocity_m * setpoint.state().getHeadingRate();

        ChassisSpeeds u_FF = new ChassisSpeeds(vx, vy, omega);
        t.log(Level.DEBUG, kName, "u_FF", u_FF);
        return u_FF;
    }

    public static ChassisSpeeds feedback(
            Pose2d currentPose,
            TimedPose setpoint,
            double kPCart,
            double kPTheta) {
        final Twist2d positionError = getErrorTwist(currentPose, setpoint);
        t.log(Level.DEBUG, kName, "errorTwist", positionError);
        ChassisSpeeds u_FB = new ChassisSpeeds(
                kPCart * positionError.dx,
                kPCart * positionError.dy,
                kPTheta * positionError.dtheta);
        t.log(Level.DEBUG, kName, "u_FB", u_FB);
        return u_FB;
    }

    public static ChassisSpeeds velocityFeedback(
            Pose2d currentPose,
            TimedPose setpoint,
            ChassisSpeeds currentRobotRelativeVelocity,
            final double kPCartV,
            final double kPThetaV) {
        final ChassisSpeeds velocityError = getVelocityError(
                currentPose,
                setpoint,
                currentRobotRelativeVelocity);
        t.log(Level.TRACE, kName, "velocityError", velocityError);
        final ChassisSpeeds u_VFB = new ChassisSpeeds(
                kPCartV * velocityError.vxMetersPerSecond,
                kPCartV * velocityError.vyMetersPerSecond,
                kPThetaV * velocityError.omegaRadiansPerSecond);
        t.log(Level.TRACE, kName, "u_VFB", u_VFB);
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
    public static ChassisSpeeds fullFeedback(
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

    static ChassisSpeeds getVelocityError(
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

    private DriveMotionControllerUtil() {
        //
    }

}
