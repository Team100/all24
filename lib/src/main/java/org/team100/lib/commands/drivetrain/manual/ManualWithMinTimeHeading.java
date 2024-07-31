package org.team100.lib.commands.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.HeadingLatch;
import org.team100.lib.controller.MinTimeController;
import org.team100.lib.controller.State100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Function that supports manual cartesian control, and both manual and locked
 * rotational control.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 */
public class ManualWithMinTimeHeading implements FieldRelativeDriver {
    private static final double kDtSec = 0.02;
    private final SupplierLogger m_logger;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Gyro m_gyro;
    /** Absolute input supplier, null if free */
    private final Supplier<Rotation2d> m_desiredRotation;
    private final HeadingLatch m_latch;
    private final MinTimeController m_controller;
    private final LinearFilter m_outputFilter;

    // package private for testing
    Rotation2d m_goal = null;
    State100 m_thetaSetpoint = null;

    /**
     * 
     * @param parent
     * @param swerveKinodynamics
     * @param gyro
     * @param desiredRotation    absolute input supplier, null if free. usually
     *                           POV-derived.
     * @param thetaController
     * @param omegaController
     */
    public ManualWithMinTimeHeading(
            SupplierLogger parent,
            SwerveKinodynamics swerveKinodynamics,
            Gyro gyro,
            Supplier<Rotation2d> desiredRotation) {
        m_swerveKinodynamics = swerveKinodynamics;
        m_gyro = gyro;
        m_desiredRotation = desiredRotation;
        m_logger = parent.child(this);
        m_latch = new HeadingLatch();
        m_outputFilter = LinearFilter.singlePoleIIR(0.01, 0.02);

        // these parameters are total guesses
        m_controller = new MinTimeController(
                parent,
                MathUtil::angleModulus,
                15, // maxV
                12, // switchingA
                9, // weakG
                20, // strongI
                0.01, // tolerance
                0.1, // finish
                new double[] { 5.0, 0.5 });
    }

    public void reset(Pose2d currentPose) {
        m_goal = null;
        m_latch.unlatch();
        updateSetpoint(currentPose.getRotation().getRadians(), getYawRateNWURad_S());
    }

    private double getYawRateNWURad_S() {
        return m_gyro.getYawRateNWU();
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
        double yawMeasurement = state.theta().x();
        // not sure which is better
        double yawRate = state.theta().v();
        // double yawRate = getYawRateNWURad_S();

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
                Math100.getMinDistance(yawMeasurement, m_goal.getRadians()));

        // if this is the first run since the latch, then the setpoint should be
        // whatever the measurement is
        // min-time doesn't use this
        // if (m_thetaSetpoint == null) {
        // // TODO: to avoid overshoot, maybe pick a setpoint that is feasible without
        // // overshoot?
        // // updateSetpoint(headingMeasurement, headingRate);
        // m_thetaSetpoint = state.theta();
        // }

        // use the modulus closest to the measurement
        // m_thetaSetpoint = new State100(
        // Math100.getMinDistance(headingMeasurement, m_thetaSetpoint.x()),
        // m_thetaSetpoint.v());

        // in snap mode we take dx and dy from the user, and use the profile for dtheta.
        // the omega goal in snap mode is always zero.
        State100 goalState = new State100(
                Math100.getMinDistance(yawMeasurement, m_goal.getRadians()), 0);

        m_thetaSetpoint = m_controller.calculate(kDtSec, state.theta(), goalState);

        // the snap overrides the user input for omega.
        final double thetaFF = getThetaFF();

        
        final double omega = MathUtil.clamp(
                thetaFF,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        FieldRelativeVelocity twistWithSnapM_S = new FieldRelativeVelocity(twistM_S.x(), twistM_S.y(), omega);

        m_logger.logString(Level.TRACE, "mode", () -> "snap");
        m_logger.logDouble(Level.TRACE, "goal/theta", () -> m_goal.getRadians());
        m_logger.logState100(Level.TRACE, "setpoint/theta", () -> m_thetaSetpoint);
        m_logger.logDouble(Level.TRACE, "measurement/theta", () -> yawMeasurement);
        m_logger.logDouble(Level.TRACE, "measurement/omega", () -> yawRate);
        m_logger.logDouble(Level.TRACE, "error/theta", () -> m_thetaSetpoint.x() - yawMeasurement);
        m_logger.logDouble(Level.TRACE, "error/omega", () -> m_thetaSetpoint.v() - yawRate);
        m_logger.logDouble(Level.TRACE, "goal_error/theta", () -> m_thetaSetpoint.x() - goalState.x());
        m_logger.logDouble(Level.TRACE, "goal_error/omega", () -> m_thetaSetpoint.v() - goalState.v());
        m_logger.logDouble(Level.TRACE, "thetaFF", () -> thetaFF);
        m_logger.logDouble(Level.TRACE, "output/omega", () -> omega);

        // desaturate the end result to feasibility by preferring the rotation over
        // translation
        twistWithSnapM_S = m_swerveKinodynamics.preferRotation(twistWithSnapM_S);
        return twistWithSnapM_S;
    }

    private double getThetaFF() {
        double thetaFF = m_thetaSetpoint.v();

        if (Experiments.instance.enabled(Experiment.UseThetaFilter)) {
            // output filtering to prevent oscillation due to delay
            thetaFF = m_outputFilter.calculate(thetaFF);
        }
        if (Math.abs(thetaFF) < 0.05) {
            thetaFF = 0;
        }
        return thetaFF;
    }

    @Override
    public String getGlassName() {
        return "ManualWithMinTimeHeading";
    }

}
