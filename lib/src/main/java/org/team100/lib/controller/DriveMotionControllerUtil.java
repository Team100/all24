package org.team100.lib.controller;

import java.util.Optional;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveMotionControllerUtil {
    private static final double kPathk = 2.4;
    private static final double kPathKTheta = 2.4;
    private static final String m_name = Names.name(DriveMotionControllerUtil.class);

    private static final Telemetry t = Telemetry.get();

    private DriveMotionControllerUtil() {
        //
    }

    private static Rotation2d direction(Pose2d measurement, TimedPose setpoint) {
        // Field relative
        Optional<Rotation2d> course = setpoint.state().getCourse();
        Rotation2d motion_direction = course.isPresent() ? course.get() : GeometryUtil.kRotationZero;
        // Adjust course by ACTUAL heading rather than planned to decouple heading and
        // translation errors.
        motion_direction = measurement.getRotation().unaryMinus().rotateBy(motion_direction);
        t.log(Level.TRACE, m_name, "motion direction", motion_direction);
        return motion_direction;
    }

    public static ChassisSpeeds feedforward(Pose2d current_state, TimedPose setpoint) {
        final double velocity_m = setpoint.velocityM_S();
        t.log(Level.TRACE, m_name, "setpoint velocity", velocity_m);

        Rotation2d motion_direction = direction(current_state, setpoint);

        double vx = motion_direction.getCos() * velocity_m;
        double vy = motion_direction.getSin() * velocity_m;
        // heading rate is rad/m of movement, so multiply by m/s to get rad/s
        double omega = velocity_m * setpoint.state().getHeadingRate();

        ChassisSpeeds u_FF = new ChassisSpeeds(vx, vy, omega);
        t.log(Level.TRACE, m_name, "u_FF", u_FF);
        return u_FF;
    }

    public static ChassisSpeeds feedback(Pose2d current_state, TimedPose setpoint, double kPCart, double kPTheta) {
        final Pose2d error = getError(current_state, setpoint);
        t.log(Level.TRACE, m_name, "error", error);
        Twist2d errorTwist = GeometryUtil.kPoseZero.log(error);
        t.log(Level.TRACE, m_name, "errorTwist", errorTwist);
        ChassisSpeeds u_FB = new ChassisSpeeds(
                kPCart * errorTwist.dx,
                kPCart * errorTwist.dy,
                kPTheta * errorTwist.dtheta);
        t.log(Level.TRACE, m_name, "u_FB", u_FB);
        return u_FB;

        
    }

    public static ChassisSpeeds fullFeedback(Pose2d current_state, TimedPose setpoint, Twist2d currentVelocity, double kPCart, double kPTheta, double kPCartV, double kPThetaV) {
        final double velocity_m = setpoint.velocityM_S();
        Rotation2d motion_direction = direction(current_state, setpoint);

        double vx = motion_direction.getCos() * velocity_m;
        double vy = motion_direction.getSin() * velocity_m;
        // heading rate is rad/m of movement, so multiply by m/s to get rad/s
        double omega = velocity_m * setpoint.state().getHeadingRate();

        Twist2d errorVelocityTwist = new Twist2d(
            vx - currentVelocity.dx,
            vy - currentVelocity.dy,
            omega - currentVelocity.dtheta
        );

        final Pose2d error = getError(current_state, setpoint);
        t.log(Level.TRACE, m_name, "error", error);
        Twist2d errorTwist = GeometryUtil.kPoseZero.log(error);
        t.log(Level.TRACE, m_name, "errorTwist", errorTwist);

        ChassisSpeeds u_XFB = new ChassisSpeeds(
                kPathk * errorTwist.dx,
                kPathk * errorTwist.dy,
                kPathKTheta * errorTwist.dtheta);

        ChassisSpeeds u_VFB = new ChassisSpeeds(
                kPCartV * errorVelocityTwist.dx,
                kPCartV * errorVelocityTwist.dy,
                kPTheta * errorVelocityTwist.dtheta
        );

        t.log(Level.TRACE, m_name, "u_XFB", u_XFB);
        t.log(Level.TRACE, m_name, "u_VFB", u_VFB);

        return u_XFB.plus(u_VFB);

        
    }

    static Pose2d getError(Pose2d current_state, TimedPose mSetpoint) {
        return GeometryUtil.transformBy(GeometryUtil.inverse(current_state), mSetpoint.state().getPose());
    }

}
