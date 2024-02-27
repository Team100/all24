package org.team100.lib.motion.drivetrain.manual;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.FieldRelativeDriver;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.State100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveState;
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
import edu.wpi.first.math.geometry.Twist2d;

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
    private final Telemetry t = Telemetry.get();
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final HeadingInterface m_heading;
    private final Supplier<Optional<Translation2d>> m_target;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    private final String m_name;
    private final TrapezoidProfile100 m_profile;
    State100 m_thetaSetpoint;
    Translation2d m_ball;
    Translation2d m_ballV;
    BooleanSupplier m_trigger;
    Pose2d m_prevPose;

    public FieldManualWithNoteRotation(
            String parent,
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
        m_name = Names.append(parent, this);
        m_trigger = trigger;
        Constraints100 c = new Constraints100(
                swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed,
                swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed);
        m_profile = new TrapezoidProfile100(c, 0.01);
    }

    @Override
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
     * @return feasible field-relative velocity in m/s and rad/s
     */
    @Override
    public Twist2d apply(SwerveState state, Twist2d input) {
        // clip the input to the unit circle
        double omega;
        Twist2d clipped = DriveUtil.clampTwist(input, 1.0);
        Optional<Translation2d> target = m_target.get();
        Twist2d scaledInput = DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        if (!target.isPresent()) {
            Twist2d twistWithLockM_S = new Twist2d(scaledInput.dx, scaledInput.dy, scaledInput.dtheta);

            // desaturate to feasibility by preferring the rotational velocity.
            twistWithLockM_S = m_swerveKinodynamics.preferRotation(twistWithLockM_S);
            m_prevPose = state.pose();
            return twistWithLockM_S;
        }
        Rotation2d currentRotation = state.pose().getRotation();
        double headingRate = m_heading.getHeadingRateNWU();
        Translation2d currentTranslation = state.pose().getTranslation();
        Rotation2d bearing = bearing(currentTranslation, target.get());

        // take the short path
        double measurement = currentRotation.getRadians();
        bearing = new Rotation2d(
                Math100.getMinDistance(measurement, bearing.getRadians()));

        // make sure the setpoint uses the modulus close to the measurement.
        m_thetaSetpoint = new State100(
                Math100.getMinDistance(measurement, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());

        // the goal omega should match the target's apparent motion
        double targetMotion = targetMotion(state, target.get());
        t.log(Level.DEBUG, m_name, "apparent motion", targetMotion);

        State100 goal = new State100(bearing.getRadians(), targetMotion);

        m_thetaSetpoint = m_profile.calculate(kDtSec, m_thetaSetpoint, goal);

        // this is user input scaled to m/s and rad/s

        double thetaFF = m_thetaSetpoint.v();

        double thetaFB = m_thetaController.calculate(measurement, m_thetaSetpoint.x());
        t.log(Level.DEBUG, m_name, "theta/setpoint", m_thetaSetpoint);
        t.log(Level.DEBUG, m_name, "theta/measurement", measurement);
        t.log(Level.DEBUG, m_name, "theta/error", m_thetaController.getPositionError());
        t.log(Level.DEBUG, m_name, "theta/fb", thetaFB);
        double omegaFB = m_omegaController.calculate(headingRate, m_thetaSetpoint.v());
        t.log(Level.DEBUG, m_name, "omega/reference", m_thetaSetpoint);
        t.log(Level.DEBUG, m_name, "omega/measurement", headingRate);
        t.log(Level.DEBUG, m_name, "omega/error", m_omegaController.getPositionError());
        t.log(Level.DEBUG, m_name, "omega/fb", omegaFB);

        omega = MathUtil.clamp(
                thetaFF + thetaFB + omegaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // this name needs to be exactly "/field/target" for glass.
        t.log(Level.DEBUG, "field", "target", new double[] {
                target.get().getX(),
                target.get().getY(),
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
            t.log(Level.DEBUG, "field", "ball", new double[] {
                    m_ball.getX(),
                    m_ball.getY(),
                    0 });
        }
        Twist2d twistWithLockM_S = new Twist2d(scaledInput.dx, scaledInput.dy, omega);

        // desaturate to feasibility by preferring the rotational velocity.
        twistWithLockM_S = m_swerveKinodynamics.preferRotation(twistWithLockM_S);
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
    static Rotation2d bearing(Translation2d robot, Translation2d target) {
        return new Rotation2d(target.minus(robot).getAngle().getRadians() + Math.PI);
    }

    /**
     * Apparent motion of the target, NWU rad/s.
     * 
     * The theta profile goal is to move at this rate, i.e. tracking the apparent
     * movement.
     */
    static double targetMotion(SwerveState state, Translation2d target) {
        Translation2d robot = state.pose().getTranslation();
        Translation2d translation = target.minus(robot);
        double range = translation.getNorm();
        Rotation2d bearing = translation.getAngle();
        Rotation2d course = state.translation().getAngle();
        Rotation2d relativeBearing = bearing.minus(course);
        double speed = GeometryUtil.norm(state.twist());
        return speed * relativeBearing.getSin() / range;
    }

}
