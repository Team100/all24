package org.team100.lib.commands.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.HeadingLatch;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.State100Logger;
import org.team100.lib.logging.LoggerFactory.StringLogger;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.state.State100;
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
    // don't try to go full speed
    private static final double PROFILE_SPEED = 0.5;
    // accelerate gently to avoid upset
    private static final double PROFILE_ACCEL = 0.5;
    private static final double OMEGA_FB_DEADBAND = 0.1;
    private static final double THETA_FB_DEADBAND = 0.1;
    private final SwerveKinodynamics m_swerveKinodynamics;
    // TODO: get rid of this, use the state estimator instead
    private final Gyro m_gyro;
    /** Absolute input supplier, null if free */
    private final Supplier<Rotation2d> m_desiredRotation;
    private final HeadingLatch m_latch;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    private final LinearFilter m_outputFilter;

    // LOGGERS
    private final StringLogger m_log_mode;
    private final DoubleLogger m_log_max_speed;
    private final DoubleLogger m_log_max_accel;
    private final DoubleLogger m_log_goal_theta;
    private final State100Logger m_log_setpoint_theta;
    private final DoubleLogger m_log_measurement_theta;
    private final DoubleLogger m_log_measurement_omega;
    private final DoubleLogger m_log_error_theta;
    private final DoubleLogger m_log_error_omega;
    private final DoubleLogger m_log_theta_FF;
    private final DoubleLogger m_log_theta_FB;
    private final DoubleLogger m_log_omega_FB;
    private final DoubleLogger m_log_output_omega;

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
    public ManualWithProfiledHeading(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Gyro gyro,
            Supplier<Rotation2d> desiredRotation,
            PIDController thetaController,
            PIDController omegaController) {
        LoggerFactory child = parent.child(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_gyro = gyro;
        m_desiredRotation = desiredRotation;
        m_thetaController = thetaController;
        m_omegaController = omegaController;
        m_latch = new HeadingLatch();
        m_outputFilter = LinearFilter.singlePoleIIR(0.01, TimedRobot100.LOOP_PERIOD_S);
        m_log_mode = child.stringLogger(Level.TRACE, "mode");
        m_log_max_speed = child.doubleLogger(Level.TRACE, "maxSpeedRad_S");
        m_log_max_accel = child.doubleLogger(Level.TRACE, "maxAccelRad_S2");
        m_log_goal_theta = child.doubleLogger(Level.TRACE, "goal/theta");
        m_log_setpoint_theta = child.state100Logger(Level.TRACE, "setpoint/theta");
        m_log_measurement_theta = child.doubleLogger(Level.TRACE, "measurement/theta");
        m_log_measurement_omega = child.doubleLogger(Level.TRACE, "measurement/omega");
        m_log_error_theta = child.doubleLogger(Level.TRACE, "error/theta");
        m_log_error_omega = child.doubleLogger(Level.TRACE, "error/omega");
        m_log_theta_FF = child.doubleLogger(Level.TRACE, "thetaFF");
        m_log_theta_FB = child.doubleLogger(Level.TRACE, "thetaFB");
        m_log_omega_FB = child.doubleLogger(Level.TRACE, "omegaFB");
        m_log_output_omega = child.doubleLogger(Level.TRACE, "output/omega");
    }

    public void reset(Pose2d currentPose) {
        m_goal = null;
        m_latch.unlatch();
        m_thetaController.reset();
        m_omegaController.reset();
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
     * @param state    current drivetrain state from the pose estimator
     * @param twist1_1 control units, [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    public FieldRelativeVelocity apply(SwerveState state, DriverControl.Velocity twist1_1) {
        final FieldRelativeVelocity control = clipAndScale(twist1_1);

        final double yawMeasurement = state.theta().x();
        final double yawRateMeasurement = state.theta().v();

        final TrapezoidProfile100 m_profile = makeProfile(control, yawRateMeasurement);

        Rotation2d pov = m_desiredRotation.get();
        m_goal = m_latch.latchedRotation(
                m_profile.getMaxAcceleration(),
                state.theta(),
                pov,
                control.theta());
        if (m_goal == null) {
            // we're not in snap mode, so it's pure manual
            // in this case there is no setpoint
            m_thetaSetpoint = null;
            m_log_mode.log(() -> "free");
            // desaturate to feasibility
            return m_swerveKinodynamics.analyticDesaturation(control);
        }

        // take the short path
        m_goal = new Rotation2d(
                Math100.getMinDistance(yawMeasurement, m_goal.getRadians()));

        // if this is the first run since the latch, then the setpoint should be
        // whatever the measurement is
        if (m_thetaSetpoint == null) {
            updateSetpoint(yawMeasurement, yawRateMeasurement);
        }

        // use the modulus closest to the measurement
        m_thetaSetpoint = new State100(
                Math100.getMinDistance(yawMeasurement, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());

        // in snap mode we take dx and dy from the user, and use the profile for dtheta.
        // the omega goal in snap mode is always zero.
        State100 goalState = new State100(m_goal.getRadians(), 0);

        m_thetaSetpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_thetaSetpoint, goalState);

        // the snap overrides the user input for omega.
        double thetaFF = m_thetaSetpoint.v();

        final double thetaFB = getThetaFB(yawMeasurement);

        final double omegaFB = getOmegaFB(yawRateMeasurement);

        double omega = MathUtil.clamp(
                thetaFF + thetaFB + omegaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        FieldRelativeVelocity twistWithSnapM_S = new FieldRelativeVelocity(control.x(), control.y(), omega);

        m_log_mode.log(() -> "snap");
        m_log_goal_theta.log(m_goal::getRadians);
        m_log_setpoint_theta.log(() -> m_thetaSetpoint);
        m_log_measurement_theta.log(() -> yawMeasurement);
        m_log_measurement_omega.log(() -> yawRateMeasurement);
        m_log_error_theta.log(() -> m_thetaSetpoint.x() - yawMeasurement);
        m_log_error_omega.log(() -> m_thetaSetpoint.v() - yawRateMeasurement);
        m_log_theta_FF.log(() -> thetaFF);
        m_log_theta_FB.log(() -> thetaFB);
        m_log_omega_FB.log(() -> omegaFB);
        m_log_output_omega.log(() -> omega);

        // desaturate the end result to feasibility by preferring the rotation over
        // translation
        twistWithSnapM_S = m_swerveKinodynamics.preferRotation(twistWithSnapM_S);
        return twistWithSnapM_S;
    }

    public FieldRelativeVelocity clipAndScale(DriverControl.Velocity twist1_1) {
        // clip the input to the unit circle
        DriverControl.Velocity clipped = DriveUtil.clampTwist(twist1_1, 1.0);
        // scale to max in both translation and rotation
        return DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
    }

    /**
     * the profile has no state and is ~free to instantiate so make a new one every
     * time. the max speed adapts to the observed speed (plus a little).
     * the max speed should be half of the absolute max, to compromise
     * translation and rotation, unless the actual translation speed is less, in
     * which case we can rotate faster.
     */
    public TrapezoidProfile100 makeProfile(FieldRelativeVelocity twistM_S, double yawRate) {
        // how fast do we want to go?
        double xySpeed = twistM_S.norm();
        // fraction of the maximum speed
        double xyRatio = Math.min(1, xySpeed / m_swerveKinodynamics.getMaxDriveVelocityM_S());
        // fraction left for rotation
        double oRatio = 1 - xyRatio;
        // actual speed is at least half
        double kRotationSpeed = Math.max(0.5, oRatio);

        double maxSpeedRad_S = Math.max(Math.abs(yawRate) + 0.001,
                m_swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed) * PROFILE_SPEED;

        double maxAccelRad_S2 = m_swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed * PROFILE_ACCEL;

        m_log_max_speed.log(() -> maxSpeedRad_S);
        m_log_max_accel.log(() -> maxAccelRad_S2);

        return new TrapezoidProfile100(
                maxSpeedRad_S,
                maxAccelRad_S2,
                0.01);
    }

    private double getOmegaFB(double headingRate) {
        double omegaFB = m_omegaController.calculate(headingRate, m_thetaSetpoint.v());

        if (Experiments.instance.enabled(Experiment.SnapThetaFilter)) {
            // output filtering to prevent oscillation due to delay
            omegaFB = m_outputFilter.calculate(omegaFB);
        }
        // deadband the output to prevent shivering.
        if (Math.abs(omegaFB) < OMEGA_FB_DEADBAND) {
            omegaFB = 0;
        }
        return omegaFB;
    }

    private double getThetaFB(double headingMeasurement) {
        double thetaFB = m_thetaController.calculate(headingMeasurement, m_thetaSetpoint.x());
        if (Math.abs(thetaFB) < THETA_FB_DEADBAND) {
            thetaFB = 0;
        }
        return thetaFB;
    }
}
