package org.team100.lib.commands.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.HeadingLatch;
import org.team100.lib.controller.State100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.State100Logger;
import org.team100.lib.logging.SupplierLogger2.StringSupplierLogger2;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.sensors.Gyro;
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
 * Rotation uses simple full-state feedback and that's all..
 */
public class ManualWithFullStateHeading implements FieldRelativeDriver {
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Gyro m_gyro;
    /** Absolute input supplier, null if free */
    private final Supplier<Rotation2d> m_desiredRotation;
    private final HeadingLatch m_latch;
    // feedback gains
    private final double[] m_K;
    private final LinearFilter m_outputFilter;

    private final StringSupplierLogger2 m_log_mode;
    private final DoubleSupplierLogger2 m_log_goal_theta;
    private final State100Logger m_log_setpoint_theta;
    private final DoubleSupplierLogger2 m_log_measurement_theta;
    private final DoubleSupplierLogger2 m_log_measurement_omega;
    private final DoubleSupplierLogger2 m_log_error_theta;
    private final DoubleSupplierLogger2 m_log_error_omega;
    private final DoubleSupplierLogger2 m_log_theta_FF;
    private final DoubleSupplierLogger2 m_log_theta_FB;
    private final DoubleSupplierLogger2 m_log_omega_FB;
    private final DoubleSupplierLogger2 m_log_output_omega;

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
     * @param k                  full state gains
     */
    public ManualWithFullStateHeading(
            SupplierLogger2 parent,
            SwerveKinodynamics swerveKinodynamics,
            Gyro gyro,
            Supplier<Rotation2d> desiredRotation,
            double[] k) {
        SupplierLogger2 child = parent.child(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_gyro = gyro;
        m_desiredRotation = desiredRotation;
        m_K = k;
        m_latch = new HeadingLatch();
        m_outputFilter = LinearFilter.singlePoleIIR(0.01, TimedRobot100.LOOP_PERIOD_S);
        m_log_mode = child.stringLogger(Level.TRACE, "mode");
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
        Pose2d currentPose = state.pose();

        // clip the input to the unit circle
        DriverControl.Velocity clipped = DriveUtil.clampTwist(twist1_1, 1.0);
        // scale to max in both translation and rotation
        FieldRelativeVelocity twistM_S = DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        Rotation2d currentRotation = currentPose.getRotation();
        double yawMeasurement = currentRotation.getRadians();
        double yawRate = getYawRateNWURad_S();

        Rotation2d pov = m_desiredRotation.get();
        m_goal = m_latch.latchedRotation(state.theta(), currentRotation, pov, twistM_S.theta());
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
        if (m_thetaSetpoint == null) {
            // TODO: to avoid overshoot, maybe pick a setpoint that is feasible without
            // overshoot?
            updateSetpoint(yawMeasurement, yawRate);
        }

        // in snap mode we take dx and dy from the user, and control dtheta.
        // the omega goal in snap mode is always zero.
        m_thetaSetpoint = new State100(m_goal.getRadians(), 0);

        // the snap overrides the user input for omega.
        double thetaFF = m_thetaSetpoint.v();

        double thetaError = MathUtil.angleModulus(m_thetaSetpoint.x() - yawMeasurement);
        double omegaError = m_thetaSetpoint.v() - yawRate;

        final double omegaFB = getOmegaFB(omegaError);
        final double thetaFB = getThetaFB(thetaError);

        final double omega = MathUtil.clamp(
                thetaFF + thetaFB + omegaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        FieldRelativeVelocity twistWithSnapM_S = new FieldRelativeVelocity(twistM_S.x(), twistM_S.y(), omega);

        m_log_mode.log(() -> "snap");
        m_log_goal_theta.log(m_goal::getRadians);
        m_log_setpoint_theta.log(() -> m_thetaSetpoint);
        m_log_measurement_theta.log(() -> yawMeasurement);
        m_log_measurement_omega.log(() -> yawRate);
        m_log_error_theta.log(() -> thetaError);
        m_log_error_omega.log(() -> omegaError);
        m_log_theta_FF.log(() -> thetaFF);
        m_log_theta_FB.log(() -> thetaFB);
        m_log_omega_FB.log(() -> omegaFB);
        m_log_output_omega.log(() -> omega);

        // desaturate the end result to feasibility by preferring the rotation over
        // translation
        twistWithSnapM_S = m_swerveKinodynamics.preferRotation(twistWithSnapM_S);
        return twistWithSnapM_S;
    }

    private double getOmegaFB(double omegaError) {
        double omegaFB = m_K[1] * omegaError;

        if (Experiments.instance.enabled(Experiment.UseThetaFilter)) {
            // output filtering to prevent oscillation due to delay
            omegaFB = m_outputFilter.calculate(omegaFB);
        }
        if (Math.abs(omegaFB) < 0.05) {
            omegaFB = 0;
        }
        return omegaFB;
    }

    private double getThetaFB(double thetaError) {
        double thetaFB = m_K[0] * thetaError;
        if (Math.abs(thetaFB) < 0.05) {
            thetaFB = 0;
        }
        return thetaFB;
    }
}
