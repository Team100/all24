package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.HeadingLatch;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.Constraints100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Function that supports manual cartesian control, and both manual and locked
 * rotational control.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 */
public class ManualWithHeading {
    private static final double kDtSec = 0.02;
    /**
     * Relative rotational speed. Use a moderate value to trade rotation for
     * translation
     */
    private static final double kRotationSpeed = 0.5;
    private final Telemetry t = Telemetry.get();
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final HeadingInterface m_heading;
    private final Supplier<Rotation2d> m_desiredRotation;
    private final HeadingLatch m_latch;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    private final String m_name;

    public final TrapezoidProfile100 m_profile;
    Rotation2d m_goal = null;
    State100 m_thetaSetpoint;

    public ManualWithHeading(
            SwerveKinodynamics swerveKinodynamics,
            HeadingInterface heading,
            Supplier<Rotation2d> desiredRotation,
            PIDController thetaController,
            PIDController omegaController) {
        m_swerveKinodynamics = swerveKinodynamics;
        m_heading = heading;
        m_desiredRotation = desiredRotation;
        m_thetaController = thetaController;
        m_omegaController = omegaController;
        m_name = Names.name(this);
        m_latch = new HeadingLatch();
        Constraints100 c = new Constraints100(
                swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed,
                swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed);
        m_profile = new TrapezoidProfile100(c, 0.01);
    }

    public void reset(Pose2d currentPose) {
        m_goal = null;
        m_latch.unlatch();
        m_thetaSetpoint = new State100(currentPose.getRotation().getRadians(), m_heading.getHeadingRateNWU());
        m_thetaController.reset();
        m_omegaController.reset();
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds, and then desaturates to a feasible holonomic velocity.
     * 
     * If you touch the POV and not the twist rotation, it remembers the POV. if you
     * use the twist rotation, it forgets and just uses that.
     * 
     * Desaturation prefers the rotational profile completely in the snap case, and
     * normally in the non-snap case.
     * 
     * This uses a fixed dt = 0.02 for the profile.
     * 
     * @param currentPose from the pose estimator
     * @param twist1_1    control units, [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    public Twist2d apply(Pose2d currentPose, Twist2d twist1_1) {
        // clip the input to the unit circle
        Twist2d clipped = DriveUtil.clampTwist(twist1_1, 1.0);

        Rotation2d currentRotation = currentPose.getRotation();
        double headingMeasurement = currentRotation.getRadians();
        double headingRate = m_heading.getHeadingRateNWU();

        Rotation2d pov = m_desiredRotation.get();
        m_goal = m_latch.latchedRotation(pov, clipped);
        if (m_goal == null) {
            // we're not in snap mode, so it's pure manual
            t.log(Level.DEBUG, m_name, "mode", "free");

            // scale to max in both translation and rotation
            Twist2d twistM_S = DriveUtil.scale(
                    clipped,
                    m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                    m_swerveKinodynamics.getMaxAngleSpeedRad_S());

            // desaturate to feasibility
            return m_swerveKinodynamics.analyticDesaturation(twistM_S);
        }

        // take the short path
        m_goal = new Rotation2d(
                Math100.getMinDistance(headingMeasurement, m_goal.getRadians()));

        // use the modulus cloest to the measurement
        m_thetaSetpoint = new State100(
                Math100.getMinDistance(headingMeasurement, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());

        // in snap mode we take dx and dy from the user, and use the profile for dtheta.
        // the omega goal in snap mode is always zero.
        State100 goalState = new State100(m_goal.getRadians(), 0);
        m_thetaSetpoint = m_profile.calculate(kDtSec, m_thetaSetpoint, goalState);

        // this is user input
        Twist2d twistM_S = DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        
        // the snap overrides the user input for omega.
        double thetaFF = m_thetaSetpoint.v();

        double thetaFB = m_thetaController.calculate(headingMeasurement, m_thetaSetpoint.x());

        double omegaFB = m_omegaController.calculate(headingRate, m_thetaSetpoint.v());

        double omega = MathUtil.clamp(
                thetaFF + thetaFB + omegaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        Twist2d twistWithSnapM_S = new Twist2d(twistM_S.dx, twistM_S.dy, omega);

        t.log(Level.DEBUG, m_name, "mode", "snap");
        t.log(Level.DEBUG, m_name, "theta setpoint", m_thetaSetpoint);
        t.log(Level.DEBUG, m_name, "measurement/theta", headingMeasurement);
        t.log(Level.DEBUG, m_name, "measurement/omega", headingRate);
        t.log(Level.DEBUG, m_name, "error/theta", m_thetaSetpoint.x() - headingMeasurement);
        t.log(Level.DEBUG, m_name, "error/omega", m_thetaSetpoint.v() - headingRate);

        // desaturate the end result to feasibility by preferring the rotation over
        // translation
        twistWithSnapM_S = m_swerveKinodynamics.preferRotation(twistWithSnapM_S);
        return twistWithSnapM_S;
    }
}
