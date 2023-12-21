package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.HeadingLatch;
import org.team100.lib.motion.drivetrain.SpeedLimits;
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
import edu.wpi.first.wpilibj.Timer;

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
    private final Telemetry t = Telemetry.get();

    private final HeadingInterface m_heading;
    private final SpeedLimits m_speedLimits;
    private final Timer m_timer;
    private final Supplier<Rotation2d> m_desiredRotation;
    private final HeadingLatch m_latch;
    private final PIDController m_thetaController;

    public Rotation2d m_currentDesiredRotation = null;
    public final TrapezoidProfile m_profile;
    TrapezoidProfile.State m_setpoint;
    private double prevTime;

    public ManualWithHeading(
            SpeedLimits speedLimits,
            HeadingInterface heading,
            Supplier<Rotation2d> desiredRotation,
            PIDController thetaController) {
        m_heading = heading;
        m_speedLimits = speedLimits;
        m_timer = new Timer();
        m_desiredRotation = desiredRotation;
        m_thetaController = thetaController;
        m_latch = new HeadingLatch();
        TrapezoidProfile.Constraints c = new TrapezoidProfile.Constraints(
                speedLimits.angleSpeedRad_S, speedLimits.angleAccelRad_S2);
        m_profile = new TrapezoidProfile(c);
    }

    public void reset(Pose2d currentPose) {
        m_currentDesiredRotation = null;
        m_timer.restart();
        prevTime = 0;
        m_latch.unlatch();
        m_setpoint = new TrapezoidProfile.State(currentPose.getRotation().getRadians(), 0);
    }

    public Twist2d apply(Pose2d currentPose, Twist2d twist1_1) {

        Rotation2d pov = m_desiredRotation.get();
        Rotation2d latchedPov = m_latch.latchedRotation(pov, twist1_1);
        if (latchedPov == null) {
            // we're not in snap mode, so it's pure manual
            m_currentDesiredRotation = null;
            t.log(Level.DEBUG, "/ManualWithHeading/mode", "free");
            return DriveUtil.scale(twist1_1, m_speedLimits.speedM_S, m_speedLimits.angleSpeedRad_S);
        }

        // if the desired rotation has changed, update the profile.
        if (!latchedPov.equals(m_currentDesiredRotation)) {
            m_currentDesiredRotation = latchedPov;
            // m_profile = updateProfile(m_speedLimits, currentPose.getRotation(),
            // m_heading.getHeadingRateNWU(),
            // latchedPov);
            m_timer.restart();
            prevTime = 0;
        }

        double now = m_timer.get();
        // in snap mode we take dx and dy from the user, and use the profile for dtheta.
        double dt = now - prevTime;
        m_setpoint = m_profile.calculate(dt,
                new TrapezoidProfile.State(latchedPov.getRadians(), 0),
                m_setpoint);
        prevTime = now;

        // this is user input
        Twist2d twistM_S = DriveUtil.scale(twist1_1, m_speedLimits.speedM_S, m_speedLimits.angleSpeedRad_S);
        // the snap overrides the user input for omega.
        double thetaFF = m_setpoint.velocity;

        Rotation2d currentRotation = currentPose.getRotation();

        double thetaFB = m_thetaController.calculate(currentRotation.getRadians(), m_setpoint.position);

        double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_speedLimits.angleSpeedRad_S,
                m_speedLimits.angleSpeedRad_S);
        Twist2d twistWithSnapM_S = new Twist2d(twistM_S.dx, twistM_S.dy, omega);

        double headingMeasurement = currentPose.getRotation().getRadians();
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
