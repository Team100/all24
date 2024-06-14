package org.team100.frc2024.motion.drivetrain.manual;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.commands.drivetrain.FieldRelativeDriver;
import org.team100.lib.controller.State100;
import org.team100.lib.geometry.TargetUtil;
import org.team100.lib.geometry.Vector2d;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.Constraints100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Names;

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
public class ManualWithShooterLock implements FieldRelativeDriver {
    private static final double kBallVelocityM_S = 5;
    private static final double kDtSec = 0.02;
    /**
     * Relative rotational speed. Use a moderate value to trade rotation for
     * translation
     */
    private static final double kRotationSpeed = 0.5;
    private final Telemetry t = Telemetry.get();
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final HeadingInterface m_heading;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    private final String m_name;
    private final TrapezoidProfile100 m_profile;
    private State100 m_thetaSetpoint;
    private Translation2d m_ball;
    private Translation2d m_ballV;
    private BooleanSupplier m_trigger;
    private Pose2d m_prevPose;
    private State100 prevGoal;
    private boolean isAligned;
    private boolean first;
    public ManualWithShooterLock(
            String parent,
            SwerveKinodynamics swerveKinodynamics,
            HeadingInterface heading,
            PIDController thetaController,
            PIDController omegaController) {
        m_swerveKinodynamics = swerveKinodynamics;
        m_heading = heading;
        m_thetaController = thetaController;
        m_omegaController = omegaController;
        isAligned = false;
        m_name = Names.append(parent, this);
        m_trigger = () -> false;
        Constraints100 c = new Constraints100(
                swerveKinodynamics.getMaxAngleSpeedRad_S(),
                swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed/4);
        m_profile = new TrapezoidProfile100(c, 0.01);
    }

    @Override
    public void reset(Pose2d currentPose) {
        m_thetaSetpoint = new State100(currentPose.getRotation().getRadians(), m_heading.getHeadingRateNWU());
        m_ball = null;
        first = true;
        prevGoal = new State100();
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
        Rotation2d bearing = bearing(currentTranslation, target);
        // Rotation2d bearingCorrected = aimWhileMoving(bearing, 20, state);

        t.log(Level.DEBUG, m_name, "bearing", bearing);

        // take the short path
        double measurement = currentRotation.getRadians();
        bearing = new Rotation2d(
                Math100.getMinDistance(measurement, bearing.getRadians()));

        checkBearing(bearing, currentRotation);

        t.log(Level.TRACE, m_name, "Bearing Check", bearing.minus(currentRotation).getDegrees());

        // make sure the setpoint uses the modulus close to the measurement.
        if (first) {
            m_thetaSetpoint = new State100(
            measurement,
            headingRate );
            first = false;
        } else {
        m_thetaSetpoint = new State100(
                Math100.getMinDistance(measurement, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());
        }
                

        // the goal omega should match the target's apparent motion
        double targetMotion = TargetUtil.targetMotion(state, target);
        t.log(Level.TRACE, m_name, "apparent motion", targetMotion);
        State100 goal = new State100(bearing.getRadians(), targetMotion);
        if (Math.abs(goal.x()-prevGoal.x()) < 0.05 || Math.abs(2*Math.PI - goal.x()-prevGoal.x()) < 0.05) {
            goal = new State100(prevGoal.x(), goal.v(),goal.a());
        }
        if (Math.abs(goal.v()-prevGoal.v()) < 0.05) {
            goal = new State100(goal.x(), 0,goal.a());
        }
        if (Math.abs(goal.a()) < 0.05) {
            goal = new State100(goal.x(), goal.v(),0);
        }
        m_thetaSetpoint = m_profile.calculate(kDtSec, m_thetaSetpoint, goal);

        // this is user input scaled to m/s and rad/s
        FieldRelativeVelocity scaledInput = DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        double thetaFF = m_thetaSetpoint.v();

        double thetaFB = m_thetaController.calculate(measurement, m_thetaSetpoint.x());

        if (Math.abs(thetaFB) < 0.5) {
            thetaFB = 0;
        }

        double omegaFB = m_omegaController.calculate(headingRate, m_thetaSetpoint.v());

        if (Math.abs(omegaFB) < 0.1) {
            omegaFB = 0;
        }
        t.log(Level.TRACE, m_name, "target", target);
        t.log(Level.SILENT, m_name, "theta/setpoint", m_thetaSetpoint);
        t.log(Level.SILENT, m_name, "theta/measurement", measurement);
        t.log(Level.TRACE, m_name, "theta/error", m_thetaController.getPositionError());
        t.log(Level.SILENT, m_name, "theta/fb", thetaFB);
        t.log(Level.SILENT, m_name, "omega/measurement", headingRate);
        t.log(Level.SILENT, m_name, "omega/error", m_omegaController.getPositionError());
        t.log(Level.SILENT, m_name, "omega/fb", omegaFB);
        t.log(Level.TRACE, m_name, "target motion", targetMotion);
        t.log(Level.SILENT, m_name, "goal", goal);
        prevGoal = goal;
        double omega = MathUtil.clamp(
                thetaFF ,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        FieldRelativeVelocity twistWithLockM_S = new FieldRelativeVelocity(scaledInput.x(), scaledInput.y(), omega);
        // desaturate to feasibility by preferring the rotational velocity.
        twistWithLockM_S = m_swerveKinodynamics.preferRotation(twistWithLockM_S);
        // this name needs to be exactly "/field/target" for glass.
        t.log(Level.TRACE, "field", "target", new double[] {
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
            t.log(Level.TRACE, "field", "ball", new double[] {
                    m_ball.getX(),
                    m_ball.getY(),
                    0 });
        }

        m_prevPose = state.pose();
        return twistWithLockM_S;
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