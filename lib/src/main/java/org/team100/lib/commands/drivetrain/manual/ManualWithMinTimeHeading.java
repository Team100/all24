package org.team100.lib.commands.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.HeadingLatch;
import org.team100.lib.controller.simple.MinTimeController;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.StringLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Function that supports manual cartesian control, and both manual and locked
 * rotational control.
 * 
 * Rotation uses a "min time" controller.
 * 
 * This driver does not play well with the setpoint generator: the acceleration
 * limiter makes it fall behind, which means it oscillates with high
 * amplitude and low frequency.
 * 
 * The issue is that the setpoint generator accurately models the low torque
 * available at high speed, whereas the available torque is modeled here as a
 * constant.
 * 
 * Don't use this class before implementing variable torque limits.
 */
public class ManualWithMinTimeHeading implements FieldRelativeDriver {
    /**
     * in "gentle snaps" mode this is the max omega allowed. The
     * idea is to get to the setpoint over a few seconds, so plan ahead!
     */
    private static final double GENTLE_OMEGA = Math.PI / 2;
    private final SwerveKinodynamics m_swerveKinodynamics;
    /** Absolute input supplier, null if free */
    private final Supplier<Rotation2d> m_desiredRotation;
    private final HeadingLatch m_latch;
    private final MinTimeController m_controller;
    private final LinearFilter m_outputFilter;

    private final StringLogger m_log_mode;
    private final DoubleLogger m_log_goal_theta;
    private final Control100Logger m_log_setpoint_theta;
    private final DoubleLogger m_log_measurement_theta;
    private final DoubleLogger m_log_measurement_omega;
    private final DoubleLogger m_log_error_theta;
    private final DoubleLogger m_log_error_omega;
    private final DoubleLogger m_log_goal_error_theta;
    private final DoubleLogger m_log_goal_error_omega;
    private final DoubleLogger m_log_theta_FF;
    private final DoubleLogger m_log_output_omega;
    private final DoubleLogger m_log_desat_omega;

    // package private for testing
    Rotation2d m_goal = null;
    Control100 m_thetaSetpoint = null;

    /**
     * 
     * @param parent
     * @param swerveKinodynamics
     * @param desiredRotation    absolute input supplier, null if free. usually
     *                           POV-derived.
     */
    public ManualWithMinTimeHeading(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Rotation2d> desiredRotation) {
        LoggerFactory child = parent.child(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_desiredRotation = desiredRotation;
        m_latch = new HeadingLatch();
        m_outputFilter = LinearFilter.singlePoleIIR(0.01, TimedRobot100.LOOP_PERIOD_S);

        // these parameters are total guesses
        m_controller = new MinTimeController(
                child,
                MathUtil::angleModulus,
                8, // maxV
                6, // switchingA
                4, // weakG
                10, // strongI
                0.01, // tolerance
                0.1, // finish
                new double[] { 5.0, 0.35 });
        m_log_mode = child.stringLogger(Level.TRACE, "mode");
        m_log_goal_theta = child.doubleLogger(Level.TRACE, "goal/theta");
        m_log_setpoint_theta = child.control100Logger(Level.TRACE, "setpoint/theta");
        m_log_measurement_theta = child.doubleLogger(Level.TRACE, "measurement/theta");
        m_log_measurement_omega = child.doubleLogger(Level.TRACE, "measurement/omega");
        m_log_error_theta = child.doubleLogger(Level.TRACE, "error/theta");
        m_log_error_omega = child.doubleLogger(Level.TRACE, "error/omega");
        m_log_goal_error_theta = child.doubleLogger(Level.TRACE, "goal_error/theta");
        m_log_goal_error_omega = child.doubleLogger(Level.TRACE, "goal_error/omega");
        m_log_theta_FF = child.doubleLogger(Level.TRACE, "thetaFF");
        m_log_output_omega = child.doubleLogger(Level.TRACE, "output/omega");
        m_log_desat_omega = child.doubleLogger(Level.TRACE, "desat/omega");

    }

    @Override
    public void reset(SwerveModel currentPose) {
        m_thetaSetpoint = currentPose.theta().control();
        m_goal = null;
        m_latch.unlatch();
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
     * @param state    current drivetrain state from the pose estimator
     * @param twist1_1 control units, [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    @Override
    public FieldRelativeVelocity apply(SwerveModel state, DriverControl.Velocity twist1_1) {
        // clip the input to the unit circle
        DriverControl.Velocity clipped = DriveUtil.clampTwist(twist1_1, 1.0);
        // scale to max in both translation and rotation
        FieldRelativeVelocity twistM_S = DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        double yawMeasurement = state.theta().x();
        double yawRate = state.theta().v();
        // double yawRate = getYawRateNWURad_S();

        Rotation2d pov = m_desiredRotation.get();
        m_goal = m_latch.latchedRotation(
                m_swerveKinodynamics.getMaxAngleAccelRad_S2(),
                state.theta(),
                pov,
                twistM_S.theta());
        if (m_goal == null) {
            // we're not in snap mode, so it's pure manual
            // in this case there is no setpoint
            m_thetaSetpoint = null;
            m_log_mode.log(() -> "free");
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
        Model100 goalState = new Model100(
                Math100.getMinDistance(yawMeasurement, m_goal.getRadians()), 0);

        m_thetaSetpoint = m_controller.calculate(TimedRobot100.LOOP_PERIOD_S, state.theta(), goalState);

        // the snap overrides the user input for omega.
        double thetaFF2 = getThetaFF();
        if (Experiments.instance.enabled(Experiment.SnapGentle))
            thetaFF2 = MathUtil.clamp(thetaFF2, -1.0 * GENTLE_OMEGA, GENTLE_OMEGA);

        final double thetaFF = thetaFF2;

        final double omega = MathUtil.clamp(
                thetaFF,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        m_log_mode.log(() -> "snap");
        m_log_goal_theta.log(() -> m_goal.getRadians());
        m_log_setpoint_theta.log(() -> m_thetaSetpoint);
        m_log_measurement_theta.log(() -> yawMeasurement);
        m_log_measurement_omega.log(() -> yawRate);
        m_log_error_theta.log(() -> m_thetaSetpoint.x() - yawMeasurement);
        m_log_error_omega.log(() -> m_thetaSetpoint.v() - yawRate);
        m_log_goal_error_theta.log(() -> m_thetaSetpoint.x() - goalState.x());
        m_log_goal_error_omega.log(() -> m_thetaSetpoint.v() - goalState.v());
        m_log_theta_FF.log(() -> thetaFF);
        m_log_output_omega.log(() -> omega);

        // desaturate the end result to feasibility, optionally preferring the rotation
        // over translation
        if (Experiments.instance.enabled(Experiment.SnapPreferRotation)) {
            final FieldRelativeVelocity twistWithSnapM_S = m_swerveKinodynamics
                    .preferRotation(
                            new FieldRelativeVelocity(twistM_S.x(), twistM_S.y(), omega));
            m_log_desat_omega.log(twistWithSnapM_S::theta);
            return twistWithSnapM_S;
        } else {
            final FieldRelativeVelocity twistWithSnapM_S = m_swerveKinodynamics
                    .analyticDesaturation(
                            new FieldRelativeVelocity(twistM_S.x(), twistM_S.y(), omega));
            m_log_desat_omega.log(twistWithSnapM_S::theta);
            return twistWithSnapM_S;
        }
    }

    private double getThetaFF() {
        double thetaFF = m_thetaSetpoint.v();

        if (Experiments.instance.enabled(Experiment.SnapThetaFilter)) {
            // output filtering to prevent oscillation due to delay
            thetaFF = m_outputFilter.calculate(thetaFF);
        }
        if (Math.abs(thetaFF) < 0.05) {
            thetaFF = 0;
        }
        return thetaFF;
    }
}
