package org.team100.lib.commands.drivetrain.manual;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team100.lib.controller.State100;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.TargetUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleArraySupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.State100Logger;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
public class ManualWithNoteRotation implements ChassisSpeedDriver {
    private static final double kBallVelocityM_S = 5;
    /**
     * Relative rotational speed. Use a moderate value to trade rotation for
     * translation
     */
    private static final double kRotationSpeed = 0.5;

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Gyro m_gyro;
    private final Supplier<Optional<Translation2d>> m_target;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    private final TrapezoidProfile100 m_profile;
    private final BooleanSupplier m_trigger;

    private final DoubleSupplierLogger2 m_log_apparent_motion;
    private final State100Logger m_log_theta_setpoint;
    private final DoubleSupplierLogger2 m_log_theta_measurement;
    private final DoubleSupplierLogger2 m_log_theta_error;
    private final DoubleSupplierLogger2 m_log_theta_FB;
    private final State100Logger m_log_omega_reference;
    private final DoubleSupplierLogger2 m_log_omega_measurement;
    private final DoubleSupplierLogger2 m_log_omega_error;
    private final DoubleSupplierLogger2 m_log_omega_FB;
    private final DoubleArraySupplierLogger2 m_log_target;
    private final DoubleArraySupplierLogger2 m_log_ball;

    private State100 m_thetaSetpoint;
    private Translation2d m_ball;
    private Translation2d m_ballV;
    private Pose2d m_prevPose;

    public ManualWithNoteRotation(
            SupplierLogger2 fieldLogger,
            SupplierLogger2 parent,
            SwerveKinodynamics swerveKinodynamics,
            Gyro gyro,
            Supplier<Optional<Translation2d>> target,
            PIDController thetaController,
            PIDController omegaController,
            BooleanSupplier trigger) {
        SupplierLogger2 child = parent.child(this);
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
        m_log_apparent_motion = child.doubleLogger(Level.TRACE, "apparent motion");
        m_log_theta_setpoint = child.state100Logger(Level.TRACE, "theta/setpoint");
        m_log_theta_measurement = child.doubleLogger(Level.TRACE, "theta/measurement");
        m_log_theta_error = child.doubleLogger(Level.TRACE, "theta/error");
        m_log_theta_FB = child.doubleLogger(Level.TRACE, "theta/fb");
        m_log_omega_reference = child.state100Logger(Level.TRACE, "omega/reference");
        m_log_omega_measurement = child.doubleLogger(Level.TRACE, "omega/measurement");
        m_log_omega_error = child.doubleLogger(Level.TRACE, "omega/error");
        m_log_omega_FB = child.doubleLogger(Level.TRACE, "omega/fb");
        m_log_target = fieldLogger.doubleArrayLogger(Level.TRACE, "target");
        m_log_ball = fieldLogger.doubleArrayLogger(Level.TRACE, "ball");
    }

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
     * @return feasible robot-relative velocity in m/s and rad/s
     */

    public ChassisSpeeds apply(SwerveState state, DriverControl.Velocity input) {
        // clip the input to the unit circle
        Optional<Translation2d> target = m_target.get();
        DriverControl.Velocity clipped = DriveUtil.clampTwist(input, 1.0);
        if (!target.isPresent()) {
            DriverControl.Velocity twistWithLock = new DriverControl.Velocity(clipped.x(), clipped.y(),
                    clipped.theta());

            m_prevPose = state.pose();
            ChassisSpeeds scaled = DriveUtil.scaleChassisSpeeds(
                    twistWithLock,
                    m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                    m_swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed);

            // prefer rotational velocity
            scaled = m_swerveKinodynamics.preferRotation(scaled);
            // desaturate to feasibility
            return m_swerveKinodynamics.analyticDesaturation(scaled);
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

        m_thetaSetpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_thetaSetpoint, goal);
        double thetaFF = m_thetaSetpoint.v();

        double thetaFB = m_thetaController.calculate(measurement, m_thetaSetpoint.x());
        m_log_theta_setpoint.log(() -> m_thetaSetpoint);
        m_log_theta_measurement.log(() -> measurement);
        m_log_theta_error.log(m_thetaController::getPositionError);
        m_log_theta_FB.log(() -> thetaFB);
        double omegaFB = m_omegaController.calculate(yawRate, m_thetaSetpoint.v());
        m_log_omega_reference.log(() -> m_thetaSetpoint);
        m_log_omega_measurement.log(() -> yawRate);
        m_log_omega_error.log(m_omegaController::getPositionError);
        m_log_omega_FB.log(() -> omegaFB);

        double omega = MathUtil.clamp(
                thetaFF + thetaFB + omegaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // this name needs to be exactly "/field/target" for glass.
        m_log_target.log(() -> new double[] {
                target.get().getX(),
                target.get().getY(),
                0 });

        // this is just for simulation
        if (m_trigger.getAsBoolean()) {
            m_ball = currentTranslation;
            // correct for newtonian relativity
            m_ballV = new Translation2d(kBallVelocityM_S * TimedRobot100.LOOP_PERIOD_S, currentRotation)
                    .plus(FieldRelativeDelta.delta(m_prevPose, state.pose()).getTranslation());
        }
        if (m_ball != null) {
            m_ball = m_ball.plus(m_ballV);
            // this name needs to be exactly "/field/ball" for glass.
            m_log_ball.log(() -> new double[] {
                    m_ball.getX(),
                    m_ball.getY(),
                    0 });
        }
        DriverControl.Velocity twistWithLock = new DriverControl.Velocity(clipped.x(), clipped.y(), omega);

        m_prevPose = state.pose();
        ChassisSpeeds scaled = DriveUtil.scaleChassisSpeeds(
                twistWithLock,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed);
        ChassisSpeeds withRot = new ChassisSpeeds(scaled.vxMetersPerSecond, scaled.vyMetersPerSecond,
                twistWithLock.theta());
        // prefer rotational velocity
        withRot = m_swerveKinodynamics.preferRotation(withRot);
        // desaturate to feasibility
        return m_swerveKinodynamics.analyticDesaturation(withRot);
    }
}
