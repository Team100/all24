package org.team100.lib.commands.drivetrain.manual;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team100.lib.controller.State100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.TargetUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleArraySupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.State100Logger;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Manual cartesian control, with rotational control based on a target position.
 * 
 * This is useful for shooting solutions, or for keeping the camera pointed at
 * something.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 * 
 * The targeting solution is based on bearing alone, so it won't work if the
 * robot or target is moving. That effect can be compensated, though.
 */
public class FieldManualWithNoteRotation implements FieldRelativeDriver {
    private static final double kBallVelocityM_S = 5;
    private static final double kDtSec = 0.02;
    /**
     * Relative rotational speed. Use a moderate value to trade rotation for
     * translation
     */
    private static final double kRotationSpeed = 0.5;

    private final SupplierLogger2 m_fieldLogger;
    private final SupplierLogger2 m_logger;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Gyro m_gyro;
    private final Supplier<Optional<Translation2d>> m_target;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    private final TrapezoidProfile100 m_profile;
    private final BooleanSupplier m_trigger;

    // LOGGERS
    private final DoubleSupplierLogger2 m_log_apparent_motion;
    private final DoubleArraySupplierLogger2 m_log_target;
    private final DoubleArraySupplierLogger2 m_log_ball;
    private final State100Logger m_log_theta_setpoint;
    private final DoubleSupplierLogger2 m_log_theta_measurement;
    private final DoubleSupplierLogger2 m_log_theta_error;
    private final DoubleSupplierLogger2 m_log_theta_fb;
    private final State100Logger m_log_omega_reference;
    private final DoubleSupplierLogger2 m_log_omega_measurement;
    private final DoubleSupplierLogger2 m_log_omega_error;
    private final DoubleSupplierLogger2 m_log_omega_fb;

    private State100 m_thetaSetpoint;
    private Translation2d m_ball;
    private Translation2d m_ballV;
    private Pose2d m_prevPose;

    public FieldManualWithNoteRotation(
            SupplierLogger2 fieldLogger,
            SupplierLogger2 parent,
            SwerveKinodynamics swerveKinodynamics,
            Gyro gyro,
            Supplier<Optional<Translation2d>> target,
            PIDController thetaController,
            PIDController omegaController,
            BooleanSupplier trigger) {
        m_fieldLogger = fieldLogger;
        m_log_target = m_fieldLogger.doubleArrayLogger(Level.TRACE, "target");
        m_log_ball = m_fieldLogger.doubleArrayLogger(Level.TRACE, "ball");
        m_logger = parent.child(this);
        m_log_apparent_motion = m_logger.doubleLogger(Level.TRACE, "apparent motion");
        m_log_theta_setpoint = m_logger.state100Logger(Level.TRACE, "theta/setpoint");
        m_log_theta_measurement = m_logger.doubleLogger(Level.TRACE, "theta/measurement");
        m_log_theta_error = m_logger.doubleLogger(Level.TRACE, "theta/error");
        m_log_theta_fb = m_logger.doubleLogger(Level.TRACE, "theta/fb");
        m_log_omega_reference = m_logger.state100Logger(Level.TRACE, "omega/reference");
        m_log_omega_measurement = m_logger.doubleLogger(Level.TRACE, "omega/measurement");
        m_log_omega_error = m_logger.doubleLogger(Level.TRACE, "omega/error");
        m_log_omega_fb = m_logger.doubleLogger(Level.TRACE, "omega/fb");

        m_swerveKinodynamics = swerveKinodynamics;
        m_gyro = gyro;
        m_target = target;
        m_thetaController = thetaController;
        m_omegaController = omegaController;
        m_profile = new TrapezoidProfile100(
                swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed,
                swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed,
                0.01);
        m_trigger = trigger;
    }

    @Override
    public void reset(Pose2d currentPose) {
        m_thetaSetpoint = new State100(currentPose.getRotation().getRadians(), m_gyro.getYawRateNWU());
        m_ball = null;
        m_prevPose = currentPose;
        m_thetaController.reset();
        m_omegaController.reset();
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds, and then desaturates to a feasible holonomic velocity.
     * 
     * @param state from the drivetrain
     * @param input control units [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    @Override
    public FieldRelativeVelocity apply(SwerveState state, DriverControl.Velocity input) {
        // clip the input to the unit circle
        double omega;
        DriverControl.Velocity clipped = DriveUtil.clampTwist(input, 1.0);
        Optional<Translation2d> target = m_target.get();
        FieldRelativeVelocity scaledInput = DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        if (!target.isPresent()) {
            FieldRelativeVelocity twistWithLockM_S = new FieldRelativeVelocity(scaledInput.x(), scaledInput.y(),
                    scaledInput.theta());

            // desaturate to feasibility by preferring the rotational velocity.
            twistWithLockM_S = m_swerveKinodynamics.preferRotation(twistWithLockM_S);
            m_prevPose = state.pose();
            return twistWithLockM_S;
        }
        Rotation2d currentRotation = state.pose().getRotation();
        double yawRate = m_gyro.getYawRateNWU();
        Translation2d currentTranslation = state.pose().getTranslation();
        Rotation2d bearing = TargetUtil.bearing(currentTranslation, target.get()).plus(GeometryUtil.kRotation180);

        // take the short path
        double measurement = currentRotation.getRadians();
        bearing = new Rotation2d(
                Math100.getMinDistance(measurement, bearing.getRadians()));

        // make sure the setpoint uses the modulus close to the measurement.
        m_thetaSetpoint = new State100(
                Math100.getMinDistance(measurement, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());

        // the goal omega should match the target's apparent motion
        double targetMotion = TargetUtil.targetMotion(state, target.get());
        m_log_apparent_motion.log(() -> targetMotion);

        State100 goal = new State100(bearing.getRadians(), targetMotion);

        m_thetaSetpoint = m_profile.calculate(kDtSec, m_thetaSetpoint, goal);

        // this is user input scaled to m/s and rad/s

        double thetaFF = m_thetaSetpoint.v();

        double thetaFB = m_thetaController.calculate(measurement, m_thetaSetpoint.x());
        m_log_theta_setpoint.log(() -> m_thetaSetpoint);
        m_log_theta_measurement.log(() -> measurement);
        m_log_theta_error.log(m_thetaController::getPositionError);
        m_log_theta_fb.log(() -> thetaFB);

        double omegaFB = m_omegaController.calculate(yawRate, m_thetaSetpoint.v());
        m_log_omega_reference.log(() -> m_thetaSetpoint);
        m_log_omega_measurement.log(() -> yawRate);
        m_log_omega_error.log(m_omegaController::getPositionError);
        m_log_omega_fb.log(() -> omegaFB);

        omega = MathUtil.clamp(
                thetaFF + thetaFB + omegaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // this name needs to be exactly "/field/target" for glass.
        m_log_target.log(() -> new double[] { target.get().getX(), target.get().getY(), 0 });

        // this is just for simulation
        if (m_trigger.getAsBoolean()) {
            m_ball = currentTranslation;
            // correct for newtonian relativity
            m_ballV = new Translation2d(kBallVelocityM_S * kDtSec, currentRotation)
                    .plus(FieldRelativeDelta.delta(m_prevPose, state.pose()).getTranslation());
        }
        if (m_ball != null) {
            m_ball = m_ball.plus(m_ballV);
            // this name needs to be exactly "/field/ball" for glass.
            m_log_ball.log(() -> new double[] { m_ball.getX(), m_ball.getY(), 0 });

        }
        FieldRelativeVelocity twistWithLockM_S = new FieldRelativeVelocity(scaledInput.x(), scaledInput.y(), omega);

        // desaturate to feasibility by preferring the rotational velocity.
        twistWithLockM_S = m_swerveKinodynamics.preferRotation(twistWithLockM_S);
        m_prevPose = state.pose();
        return twistWithLockM_S;
    }

    @Override
    public String getGlassName() {
        return "FieldManualWithNoteRotation";
    }

}
