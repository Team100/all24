package org.team100.frc2024.commands.drivetrain.manual;

import java.util.Optional;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.commands.drivetrain.manual.FieldRelativeDriver;
import org.team100.lib.controller.State100;
import org.team100.lib.geometry.TargetUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.telemetry.FieldLogger;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
public class ManualWithAmpLock implements FieldRelativeDriver {
    private static final double kDtSec = 0.02;
    /**
     * Relative rotational speed. Use a moderate value to trade rotation for
     * translation
     */
    private static final double kRotationSpeed = 0.5;

    private final FieldLogger m_fieldLogger;
    private final SupplierLogger m_logger;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Gyro m_gyro;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    private final TrapezoidProfile100 m_profile;

    private State100 m_thetaSetpoint;
    private Translation2d m_ball;
    private Translation2d m_ballV;

    public ManualWithAmpLock(
            FieldLogger fieldLogger,
            SupplierLogger parent,
            SwerveKinodynamics swerveKinodynamics,
            Gyro gyro,
            PIDController thetaController,
            PIDController omegaController) {
        m_fieldLogger = fieldLogger;
        m_logger = parent.child(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_gyro = gyro;
        m_thetaController = thetaController;
        m_omegaController = omegaController;
        m_profile = new TrapezoidProfile100(
                swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed,
                swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed,
                0.01);
    }

    @Override
    public void reset(Pose2d currentPose) {
        m_thetaSetpoint = new State100(currentPose.getRotation().getRadians(), m_gyro.getYawRateNWU());
        m_ball = null;
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
        DriverControl.Velocity clipped = DriveUtil.clampTwist(input, 1.0);
        Rotation2d currentRotation = state.pose().getRotation();
        double yawRate = m_gyro.getYawRateNWU();

        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (!optionalAlliance.isPresent())
            return new FieldRelativeVelocity(0, 0, 0);

        Translation2d currentTranslation = state.pose().getTranslation();
        Translation2d target = ShooterUtil.getAmpTranslation(optionalAlliance.get());
        Rotation2d bearing = TargetUtil.bearing(currentTranslation, target);

        // take the short path
        double measurement = currentRotation.getRadians();
        bearing = new Rotation2d(
                Math100.getMinDistance(measurement, bearing.getRadians()));

        // make sure the setpoint uses the modulus close to the measurement.
        m_thetaSetpoint = new State100(
                Math100.getMinDistance(measurement, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());

        // the goal omega should match the target's apparent motion
        double targetMotion = TargetUtil.targetMotion(state, target);
        m_logger.logDouble(Level.TRACE, "apparent motion", () -> targetMotion);

        State100 goal = new State100(bearing.getRadians(), targetMotion);

        m_thetaSetpoint = m_profile.calculate(kDtSec, m_thetaSetpoint, goal);

        // this is user input scaled to m/s and rad/s
        FieldRelativeVelocity scaledInput = DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        double thetaFF = m_thetaSetpoint.v();

        double thetaFB = m_thetaController.calculate(measurement, m_thetaSetpoint.x());
        m_logger.logState100(Level.TRACE, "theta/setpoint", () -> m_thetaSetpoint);
        m_logger.logDouble(Level.TRACE, "theta/measurement", () -> measurement);
        m_logger.logDouble(Level.TRACE, "theta/error", m_thetaController::getPositionError);
        m_logger.logDouble(Level.TRACE, "theta/fb", () -> thetaFB);

        double omegaFB = m_omegaController.calculate(yawRate, m_thetaSetpoint.v());
        m_logger.logState100(Level.TRACE, "omega/reference", () -> m_thetaSetpoint);
        m_logger.logDouble(Level.TRACE, "omega/measurement", () -> yawRate);
        m_logger.logDouble(Level.TRACE, "omega/error", m_omegaController::getPositionError);
        m_logger.logDouble(Level.TRACE, "omega/fb", () -> omegaFB);

        double omega = MathUtil.clamp(
                thetaFF + thetaFB + omegaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        FieldRelativeVelocity twistWithLockM_S = new FieldRelativeVelocity(scaledInput.x(), scaledInput.y(), omega);

        // desaturate to feasibility by preferring the rotational velocity.
        twistWithLockM_S = m_swerveKinodynamics.preferRotation(twistWithLockM_S);

        // this name needs to be exactly "/field/target" for glass.
        m_fieldLogger.logDoubleArray(Level.TRACE, "target", () -> new double[] {
                target.getX(),
                target.getY(),
                0 });

        if (m_ball != null) {
            m_ball = m_ball.plus(m_ballV);
            // this name needs to be exactly "/field/ball" for glass.
            m_fieldLogger.logDoubleArray(Level.TRACE, "ball", () -> new double[] {
                    m_ball.getX(),
                    m_ball.getY(),
                    0 });
        }
        return twistWithLockM_S;
    }

    @Override
    public String getGlassName() {
        return "ManualWithTargetLock";
    }

}
