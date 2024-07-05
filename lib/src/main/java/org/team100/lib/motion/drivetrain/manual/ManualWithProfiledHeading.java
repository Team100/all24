package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.FieldRelativeDriver;
import org.team100.lib.commands.drivetrain.HeadingLatch;
import org.team100.lib.controller.State100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Function that supports manual cartesian control, and both manual and locked
 * rotational control.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 */
public class ManualWithProfiledHeading implements FieldRelativeDriver {
    private static final double kDtSec = 0.02;
    private final Logger m_logger;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final HeadingInterface m_heading;
    /** Absolute input supplier, null if free */
    private final Supplier<Rotation2d> m_desiredRotation;
    private final HeadingLatch m_latch;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    private final LinearFilter m_outputFilter;

    // package private for testing
    Rotation2d m_goal = null;
    State100 m_thetaSetpoint = null;

    /**
     * 
     * @param parent
     * @param swerveKinodynamics
     * @param heading
     * @param desiredRotation    absolute input supplier, null if free. usually
     *                           POV-derived.
     * @param thetaController
     * @param omegaController
     */
    public ManualWithProfiledHeading(
            Logger parent,
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
        m_logger = parent.child(this);
        m_latch = new HeadingLatch();
        m_outputFilter = LinearFilter.singlePoleIIR(0.01, 0.02);
    }

    public void reset(Pose2d currentPose) {
        m_goal = null;
        m_latch.unlatch();
        m_thetaController.reset();
        m_omegaController.reset();
        updateSetpoint(currentPose.getRotation().getRadians(), getHeadingRateNWURad_S());
    }

    private double getHeadingRateNWURad_S() {
        return m_heading.getHeadingRateNWU();
    }

    /** Call this to keep the setpoint in sync with the manual rotation. */
    private void updateSetpoint(double x, double v) {
        m_thetaSetpoint = new State100(x, v);
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
     * @param state    current drivetrain state from the pose estimator
     * @param twist1_1 control units, [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    public FieldRelativeVelocity apply(SwerveState state, DriverControl.Velocity twist1_1) {
        Pose2d currentPose = state.pose();

        // clip the input to the unit circle
        DriverControl.Velocity clipped = DriveUtil.clampTwist(twist1_1, 1.0);
        // scale to max in both translation and rotation
        FieldRelativeVelocity twistM_S = DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        Rotation2d currentRotation = currentPose.getRotation();
        double headingMeasurement = currentRotation.getRadians();
        double headingRate = getHeadingRateNWURad_S();

        Rotation2d pov = m_desiredRotation.get();
        m_goal = m_latch.latchedRotation(state.theta(), currentRotation, pov, twistM_S.theta());
        if (m_goal == null) {
            // we're not in snap mode, so it's pure manual
            // in this case there is no setpoint
            m_thetaSetpoint = null;
            m_logger.logString(Level.TRACE, "mode", () -> "free");
            // desaturate to feasibility
            return m_swerveKinodynamics.analyticDesaturation(twistM_S);
        }

        // take the short path
        m_goal = new Rotation2d(
                Math100.getMinDistance(headingMeasurement, m_goal.getRadians()));

        // if this is the first run since the latch, then the setpoint should be
        // whatever the measurement is
        if (m_thetaSetpoint == null) {
            updateSetpoint(headingMeasurement, headingRate);
        }

        // use the modulus closest to the measurement
        m_thetaSetpoint = new State100(
                Math100.getMinDistance(headingMeasurement, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());

        // in snap mode we take dx and dy from the user, and use the profile for dtheta.
        // the omega goal in snap mode is always zero.
        State100 goalState = new State100(m_goal.getRadians(), 0);

        // the profile has no state and is ~free to instantiate so make a new one every
        // time. the max speed adapts to the observed speed (plus a little).
        // the max speed should be half of the absolute max, to compromise translation
        // and rotation, unless the actual translation speed is less, in which case we
        // can rotate faster.

        // how fast do we want to go?
        double xySpeed = twistM_S.norm();
        // fraction of the maximum speed
        double xyRatio = Math.min(1, xySpeed / m_swerveKinodynamics.getMaxDriveVelocityM_S());
        // fraction left for rotation
        double oRatio = 1 - xyRatio;
        // actual speed is at least half
        double kRotationSpeed = Math.max(0.5, oRatio);

        // finally reduce the speed to make it easier
        final double lessV = 0.5;
        // kinodynamic max A seems too high?
        final double lessA = 0.1;

        double maxSpeedRad_S = Math.max(Math.abs(headingRate) + 0.001,
                m_swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed) * lessV;
        double maxAccelRad_S2 = m_swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed * lessA;

        m_logger.logDouble(Level.TRACE, "maxSpeedRad_S", () -> maxSpeedRad_S);
        m_logger.logDouble(Level.TRACE, "maxAccelRad_S2", () -> maxAccelRad_S2);

        final TrapezoidProfile100 m_profile = new TrapezoidProfile100(
                maxSpeedRad_S,
                maxAccelRad_S2,
                0.01);

        m_thetaSetpoint = m_profile.calculate(kDtSec, m_thetaSetpoint, goalState);

        // the snap overrides the user input for omega.
        double thetaFF = m_thetaSetpoint.v();

        final double thetaFB = getThetaFB(headingMeasurement);

        final double omegaFB = getOmegaFB(headingRate);

        double omega = MathUtil.clamp(
                thetaFF + thetaFB + omegaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        FieldRelativeVelocity twistWithSnapM_S = new FieldRelativeVelocity(twistM_S.x(), twistM_S.y(), omega);

        m_logger.logString(Level.TRACE, "mode", () -> "snap");
        m_logger.logDouble(Level.TRACE, "goal/theta", m_goal::getRadians);
        m_logger.logState100(Level.TRACE, "setpoint/theta", () -> m_thetaSetpoint);
        m_logger.logDouble(Level.TRACE, "measurement/theta", () -> headingMeasurement);
        m_logger.logDouble(Level.TRACE, "measurement/omega", () -> headingRate);
        m_logger.logDouble(Level.TRACE, "error/theta", () -> m_thetaSetpoint.x() - headingMeasurement);
        m_logger.logDouble(Level.TRACE, "error/omega", () -> m_thetaSetpoint.v() - headingRate);
        m_logger.logDouble(Level.TRACE, "thetaFF", () -> thetaFF);
        m_logger.logDouble(Level.TRACE, "thetaFB", () -> thetaFB);
        m_logger.logDouble(Level.TRACE, "omegaFB", () -> omegaFB);
        m_logger.logDouble(Level.TRACE, "output/omega", () -> omega);

        // desaturate the end result to feasibility by preferring the rotation over
        // translation
        twistWithSnapM_S = m_swerveKinodynamics.preferRotation(twistWithSnapM_S);
        return twistWithSnapM_S;
    }

    private double getOmegaFB(double headingRate) {
        double omegaFB = m_omegaController.calculate(headingRate, m_thetaSetpoint.v());

        if (Experiments.instance.enabled(Experiment.UseThetaFilter)) {
            // output filtering to prevent oscillation due to delay
            omegaFB = m_outputFilter.calculate(omegaFB);
        }
        // deadband the output to prevent shivering.
        if (Math.abs(omegaFB) < 0.1) {
            omegaFB = 0;
        }
        return omegaFB;
    }

    private double getThetaFB(double headingMeasurement) {
        double thetaFB = m_thetaController.calculate(headingMeasurement, m_thetaSetpoint.x());
        if (Math.abs(thetaFB) < 0.1) {
            thetaFB = 0;
        }
        return thetaFB;
    }

    @Override
    public String getGlassName() {
        return "ManualWithProfiledHeading";
    }

}
