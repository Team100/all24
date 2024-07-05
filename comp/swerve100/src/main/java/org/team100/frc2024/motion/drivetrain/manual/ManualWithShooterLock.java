package org.team100.frc2024.motion.drivetrain.manual;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.commands.drivetrain.FieldRelativeDriver;
import org.team100.lib.controller.State100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.TargetUtil;
import org.team100.lib.geometry.Vector2d;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.FieldLogger;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
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
 * 
 * TODO: replace the two PID controllers with simpler multiplication; see
 * ManualWithFullStateHeading for an example.
 * 
 * TODO: do more investigation into jitter,
 */
public class ManualWithShooterLock implements FieldRelativeDriver {
    private static final double kBallVelocityM_S = 5;
    private static final double kDtSec = 0.02;
    /**
     * Relative rotational speed. Use a moderate value to trade rotation for
     * translation
     */
    private static final double kRotationSpeed = 0.5;

    private final Logger m_fieldLogger;
    private final Logger m_logger;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final HeadingInterface m_heading;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    private final TrapezoidProfile100 m_profile;
    // TODO: this filters the omega output since it can be noisy
    private final LinearFilter m_outputFilter;

    private State100 m_thetaSetpoint;
    private Translation2d m_ball;
    private Translation2d m_ballV;
    private BooleanSupplier m_trigger;
    private Pose2d m_prevPose;
    private boolean isAligned;
    private boolean first;

    public ManualWithShooterLock(
            FieldLogger fieldLogger,
            Logger parent,
            SwerveKinodynamics swerveKinodynamics,
            HeadingInterface heading,
            PIDController thetaController,
            PIDController omegaController) {
        m_fieldLogger = fieldLogger;
        m_logger = parent.child(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_heading = heading;
        m_thetaController = thetaController;
        m_omegaController = omegaController;
        m_profile = new TrapezoidProfile100(
                swerveKinodynamics.getMaxAngleSpeedRad_S(),
                swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed / 4,
                0.01);
        m_outputFilter = LinearFilter.singlePoleIIR(0.01, 0.02);

        isAligned = false;
        m_trigger = () -> false;
    }

    @Override
    public void reset(Pose2d currentPose) {
        m_thetaSetpoint = new State100(currentPose.getRotation().getRadians(), m_heading.getHeadingRateNWU());
        m_ball = null;
        first = true;
        m_prevPose = currentPose;
        m_thetaController.reset();
        m_omegaController.reset();
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_thetaController.setTolerance(0.05);
        m_omegaController.setTolerance(0.1);
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds, and then desaturates to a feasible holonomic velocity.
     */
    @Override
    public FieldRelativeVelocity apply(SwerveState state, DriverControl.Velocity input) {
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (!optionalAlliance.isPresent())
            return new FieldRelativeVelocity(0, 0, 0);

        // clip the input to the unit circle
        DriverControl.Velocity clipped = DriveUtil.clampTwist(input, 1.0);
        Rotation2d currentRotation = state.pose().getRotation();
        double headingRate = m_heading.getHeadingRateNWU();
        Translation2d currentTranslation = state.pose().getTranslation();
        Translation2d target = ShooterUtil.getOffsetTranslation(optionalAlliance.get());

        // take the short path
        final double measurement = currentRotation.getRadians();
        final Rotation2d bearing = new Rotation2d(
                Math100.getMinDistance(
                        measurement,
                        bearing(currentTranslation, target).getRadians()));

        // Rotation2d bearingCorrected = aimWhileMoving(bearing, 20, state);

        checkBearing(bearing, currentRotation);

        m_logger.logRotation2d(Level.TRACE, "bearing", () -> bearing);
        m_logger.logDouble(Level.TRACE, "Bearing Check", () -> bearing.minus(currentRotation).getDegrees());

        // make sure the setpoint uses the modulus close to the measurement.
        if (first) {
            m_thetaSetpoint = new State100(
                    measurement,
                    headingRate);
            first = false;
        } else {
            m_thetaSetpoint = new State100(
                    Math100.getMinDistance(measurement, m_thetaSetpoint.x()),
                    m_thetaSetpoint.v());
        }

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

        final double thetaFF = m_thetaSetpoint.v();
        final double thetaFB = getThetaFB(measurement);
        final double omegaFB = getOmegaFB(headingRate);

        m_logger.logTranslation2d(Level.TRACE, "target", () -> target);
        m_logger.logState100(Level.TRACE, "theta/setpoint", () -> m_thetaSetpoint);
        m_logger.logDouble(Level.TRACE, "theta/measurement", () -> measurement);
        m_logger.logDouble(Level.TRACE, "theta/error", m_thetaController::getPositionError);
        m_logger.logDouble(Level.TRACE, "theta/fb", () -> thetaFB);
        m_logger.logDouble(Level.TRACE, "omega/measurement", () -> headingRate);
        m_logger.logDouble(Level.TRACE, "omega/error", m_omegaController::getPositionError);
        m_logger.logDouble(Level.TRACE, "omega/fb", () -> omegaFB);
        m_logger.logDouble(Level.TRACE, "target motion", () -> targetMotion);
        m_logger.logState100(Level.TRACE, "goal", () -> goal);

        double omega = MathUtil.clamp(
                thetaFF,
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

        // this is just for simulation
        if (m_trigger.getAsBoolean()) {
            m_ball = currentTranslation;
            // correct for newtonian relativity
            m_ballV = new Translation2d(kBallVelocityM_S * kDtSec, currentRotation)
                    .plus(state.pose().minus(m_prevPose).getTranslation());
        }
        if (m_ball != null) {
            m_ball = m_ball.plus(m_ballV);
            // this name needs to be exactly "/field/ball" for glass.
            m_fieldLogger.logDoubleArray(Level.TRACE, "ball", () -> new double[] {
                    m_ball.getX(),
                    m_ball.getY(),
                    0 });
        }

        m_prevPose = state.pose();
        return twistWithLockM_S;
    }

    private double getOmegaFB(double headingRate) {
        double omegaFB = m_omegaController.calculate(headingRate, m_thetaSetpoint.v());

        if (Experiments.instance.enabled(Experiment.UseThetaFilter)) {
            // output filtering to prevent oscillation due to delay
            omegaFB = m_outputFilter.calculate(omegaFB);
        }

        if (Math.abs(omegaFB) < 0.1) {
            omegaFB = 0;
        }
        return omegaFB;
    }

    private double getThetaFB(final double measurement) {
        double thetaFB = m_thetaController.calculate(measurement, m_thetaSetpoint.x());
        if (Math.abs(thetaFB) < 0.5) {
            thetaFB = 0;
        }
        return thetaFB;
    }

    /**
     * Absolute bearing to the target.
     * 
     * The bearing is only a valid shooting solution if both the robot and the
     * target are at rest!
     * 
     * If the robot and/or target is moving, then the shooting solution needs to
     * lead or lag the target.
     */
    Rotation2d bearing(Translation2d robot, Translation2d target) {

        return target.minus(robot).getAngle();
    }

    public void checkBearing(Rotation2d bearing, Rotation2d currentRotation) {
        if (Math.abs(bearing.minus(currentRotation).getDegrees()) < 20) {
            isAligned = true;
        } else {
            isAligned = false;
        }
    }

    public boolean isAligned() {
        return isAligned;
    }

    static Rotation2d aimWhileMoving(Rotation2d bearing, double shooterVelocity, SwerveState state) {

        // its the shooter util code but robot moving vec is y velocity and angle in
        // rads is bearing

        // double angleWithoutMoving = bearing.getRadians();

        Rotation2d angleInRads = bearing;

        Vector2d stationaryRobotVector = new Vector2d(shooterVelocity, angleInRads);

        Vector2d robotMovingVector = new Vector2d(state.y().v(), 0);

        Vector2d resultingVector = Vector2d.sub(stationaryRobotVector, robotMovingVector);

        return resultingVector.getTheta();

    }

    @Override
    public String getGlassName() {
        return "ManualWithShooterLock";
    }

}