package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.HeadingLatch;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Function that supports manual cartesian control, and both manual and locked
 * rotational control.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 * 
 * TODO: add velocity feedback
 * TODO: replace the PID controller with multiplication
 */
public class ManualWithHeading {
    private static final double kDtSec = 0.02;
    private final Telemetry t = Telemetry.get();

    private final HeadingInterface m_heading;
    private final SwerveKinodynamics m_speedLimits;
    private final Supplier<Rotation2d> m_desiredRotation;
    private final HeadingLatch m_latch;
    private final PIDController m_thetaController;

    public Rotation2d m_goal = null;
    public final TrapezoidProfile m_profile;
    TrapezoidProfile.State m_setpoint;

    public ManualWithHeading(
            SwerveKinodynamics speedLimits,
            HeadingInterface heading,
            Supplier<Rotation2d> desiredRotation,
            PIDController thetaController) {
        m_heading = heading;
        m_speedLimits = speedLimits;
        m_desiredRotation = desiredRotation;
        m_thetaController = thetaController;
        m_latch = new HeadingLatch();
        TrapezoidProfile.Constraints c = new TrapezoidProfile.Constraints(
                speedLimits.getMaxAngleSpeedRad_S(), speedLimits.getMaxAngleAccelRad_S2());
        m_profile = new TrapezoidProfile(c);
    }

    public void reset(Pose2d currentPose) {
        m_goal = null;
        m_latch.unlatch();
        // TODO: include omega
        m_setpoint = new TrapezoidProfile.State(currentPose.getRotation().getRadians(), 0);
    }

    /**
     * control for fixed dt = 0.02.
     */
    public Twist2d apply(Pose2d currentPose, Twist2d twist1_1) {
        Rotation2d currentRotation = currentPose.getRotation();

        Rotation2d pov = m_desiredRotation.get();
        m_goal = m_latch.latchedRotation(pov, twist1_1);
        if (m_goal == null) {
            // we're not in snap mode, so it's pure manual
            t.log(Level.DEBUG, "/ManualWithHeading/mode", "free");
            return DriveUtil.scale(
                    twist1_1,
                    m_speedLimits.getMaxSpeedM_S(),
                    m_speedLimits.getMaxAngleSpeedRad_S());
        }

        // take the short path
        m_goal = new Rotation2d(
                MathUtil.angleModulus(m_goal.getRadians() - currentRotation.getRadians())
                        + currentRotation.getRadians());
        m_setpoint.position = MathUtil.angleModulus(m_setpoint.position - currentRotation.getRadians())
                + currentRotation.getRadians();

        // in snap mode we take dx and dy from the user, and use the profile for dtheta.
        m_setpoint = m_profile.calculate(kDtSec,
                new TrapezoidProfile.State(m_goal.getRadians(), 0),
                m_setpoint);

        // this is user input
        Twist2d twistM_S = DriveUtil.scale(
                twist1_1,
                m_speedLimits.getMaxSpeedM_S(),
                m_speedLimits.getMaxAngleSpeedRad_S());
        // the snap overrides the user input for omega.
        double thetaFF = m_setpoint.velocity;

        double thetaFB = m_thetaController.calculate(currentRotation.getRadians(), m_setpoint.position);

        double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_speedLimits.getMaxAngleSpeedRad_S(),
                m_speedLimits.getMaxAngleSpeedRad_S());
        Twist2d twistWithSnapM_S = new Twist2d(twistM_S.dx, twistM_S.dy, omega);

        double headingMeasurement = currentRotation.getRadians();
        double headingRate = m_heading.getHeadingRateNWU();

        t.log(Level.DEBUG, "/ManualWithHeading/mode", "snap");
        t.log(Level.DEBUG, "/ManualWithHeading/reference/theta", m_setpoint.position);
        t.log(Level.DEBUG, "/ManualWithHeading/reference/omega", m_setpoint.velocity);
        t.log(Level.DEBUG, "/ManualWithHeading/measurement/theta", headingMeasurement);
        t.log(Level.DEBUG, "/ManualWithHeading/measurement/omega", headingRate);
        t.log(Level.DEBUG, "/ManualWithHeading/error/theta", m_setpoint.position - headingMeasurement);
        t.log(Level.DEBUG, "/ManualWithHeading/error/omega", m_setpoint.velocity - headingRate);

        return twistWithSnapM_S;

    }
}
