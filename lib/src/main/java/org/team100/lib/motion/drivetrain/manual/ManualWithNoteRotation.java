package org.team100.lib.motion.drivetrain.manual;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.ChassisSpeedDriver;
import org.team100.lib.controller.State100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.TargetUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry;
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
    private static final double kDtSec = 0.02;
    /**
     * Relative rotational speed. Use a moderate value to trade rotation for
     * translation
     */
    private static final double kRotationSpeed = 0.5;
    private final Logger m_logger;
    private final Logger fieldLogger;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final HeadingInterface m_heading;
    private final Supplier<Optional<Translation2d>> m_target;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    private final TrapezoidProfile100 m_profile;
    State100 m_thetaSetpoint;
    Translation2d m_ball;
    Translation2d m_ballV;
    BooleanSupplier m_trigger;
    Pose2d m_prevPose;

    public ManualWithNoteRotation(
            Logger parent,
            SwerveKinodynamics swerveKinodynamics,
            HeadingInterface heading,
            Supplier<Optional<Translation2d>> target,
            PIDController thetaController,
            PIDController omegaController,
            BooleanSupplier trigger) {
        m_swerveKinodynamics = swerveKinodynamics;
        m_heading = heading;
        m_target = target;
        m_thetaController = thetaController;
        m_omegaController = omegaController;
        m_logger = parent.child(this);
        fieldLogger = Telemetry.get().fieldLogger();
        m_trigger = trigger;
        m_profile = new TrapezoidProfile100(
                swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed,
                swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed,
                0.01);
    }

    public void reset(Pose2d currentPose) {
        m_thetaSetpoint = new State100(currentPose.getRotation().getRadians(), m_heading.getHeadingRateNWU());
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
        double headingRate = m_heading.getHeadingRateNWU();
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
        m_logger.logDouble(Level.TRACE, "apparent motion", () -> targetMotion);

        State100 goal = new State100(bearing.getRadians(), targetMotion);

        m_thetaSetpoint = m_profile.calculate(kDtSec, m_thetaSetpoint, goal);
        double thetaFF = m_thetaSetpoint.v();

        double thetaFB = m_thetaController.calculate(measurement, m_thetaSetpoint.x());
        m_logger.logState100(Level.TRACE, "theta/setpoint", () -> m_thetaSetpoint);
        m_logger.logDouble(Level.TRACE, "theta/measurement", () -> measurement);
        m_logger.logDouble(Level.TRACE, "theta/error", m_thetaController::getPositionError);
        m_logger.logDouble(Level.TRACE, "theta/fb", () -> thetaFB);
        double omegaFB = m_omegaController.calculate(headingRate, m_thetaSetpoint.v());
        m_logger.logState100(Level.TRACE, "omega/reference", () -> m_thetaSetpoint);
        m_logger.logDouble(Level.TRACE, "omega/measurement", () -> headingRate);
        m_logger.logDouble(Level.TRACE, "omega/error", m_omegaController::getPositionError);
        m_logger.logDouble(Level.TRACE, "omega/fb", () -> omegaFB);

        double omega = MathUtil.clamp(
                thetaFF + thetaFB + omegaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // this name needs to be exactly "/field/target" for glass.
        fieldLogger.logDoubleArray(Level.DEBUG, "target", () -> new double[] {
                target.get().getX(),
                target.get().getY(),
                0 });

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
            fieldLogger.logDoubleArray(Level.TRACE, "ball", () -> new double[] {
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

    @Override
    public String getGlassName() {
        return "ManualWithNoteRotation";
    }

}
