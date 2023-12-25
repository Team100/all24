package org.team100.lib.motion.drivetrain.manual;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Manual cartesian control, with rotational control based on a target position.
 * 
 * This is useful for shooting solutions, or for keeping the camera pointed at
 * something.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 * 
 * TODO: add velocity feedback.
 * TODO: replace the PID controller with multiplication
 * TODO: lead the target based on velocity.
 */
public class ManualWithTargetLock {
    private static final double kBallVelocityM_S = 5;
    private static final double kDtSec = 0.02;
    private final Telemetry t = Telemetry.get();
    private final SwerveKinodynamics m_swerveKinodynamics;

    private final Supplier<Translation2d> m_target;
    private final PIDController m_thetaController;
    private final TrapezoidProfile m_profile;
    TrapezoidProfile.State m_setpoint;
    Translation2d m_ball;
    Translation2d m_ballV;
    BooleanSupplier m_trigger;
    Pose2d m_prevPose;

    public ManualWithTargetLock(
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Translation2d> target,
            PIDController thetaController,
            BooleanSupplier trigger) {
        m_swerveKinodynamics = swerveKinodynamics;
        m_target = target;
        m_thetaController = thetaController;
        m_trigger = trigger;
        TrapezoidProfile.Constraints c = new TrapezoidProfile.Constraints(
                swerveKinodynamics.getMaxAngleSpeedRad_S(),
                swerveKinodynamics.getMaxAngleAccelRad_S2());
        m_profile = new TrapezoidProfile(c);
    }

    public void reset(Pose2d currentPose) {
        // TODO: include omega
        m_setpoint = new TrapezoidProfile.State(currentPose.getRotation().getRadians(), 0);
        m_ball = null;
        m_prevPose = currentPose;
    }

    public Twist2d apply(Pose2d currentPose, Twist2d twist1_1) {
        Rotation2d currentRotation = currentPose.getRotation();
        Translation2d currentTranslation = currentPose.getTranslation();
        Translation2d target = m_target.get();
        Rotation2d goal = fieldRelativeAngleToTarget(
                currentTranslation,
                target);

        // take the short path
        goal = new Rotation2d(
                MathUtil.angleModulus(goal.getRadians() - currentRotation.getRadians())
                        + currentRotation.getRadians());
        m_setpoint.position = MathUtil.angleModulus(m_setpoint.position - currentRotation.getRadians())
                + currentRotation.getRadians();

        /** TODO: goal omega should not be zero */
        m_setpoint = m_profile.calculate(kDtSec,
                new TrapezoidProfile.State(goal.getRadians(), 0),
                m_setpoint);

        // this is user input
        Twist2d twistM_S = DriveUtil.scale(
                twist1_1,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        double thetaFF = m_setpoint.velocity;

        double thetaFB = m_thetaController.calculate(currentRotation.getRadians(), m_setpoint.position);

        double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        Twist2d twistWithLockM_S = new Twist2d(twistM_S.dx, twistM_S.dy, omega);

        t.log(Level.DEBUG, "/ManualWithTargetLock/reference/theta", m_setpoint.position);
        t.log(Level.DEBUG, "/ManualWithTargetLock/reference/omega", m_setpoint.velocity);

        t.log(Level.DEBUG, "/field/target", new double[] {
                target.getX(),
                target.getY(),
                0 });

        // this is just for simulation
        if (m_trigger.getAsBoolean()) {
            m_ball = currentTranslation;
            m_ballV = new Translation2d(kBallVelocityM_S * kDtSec, currentRotation)
                    .plus(currentPose.minus(m_prevPose).getTranslation());
        }
        if (m_ball != null) {
            m_ball = m_ball.plus(m_ballV);
            t.log(Level.DEBUG, "/field/ball", new double[] {
                    m_ball.getX(),
                    m_ball.getY(),
                    0 });
        }

        m_prevPose = currentPose;
        return twistWithLockM_S;
    }

    /** TODO: include robot velocity correction */
    static Rotation2d fieldRelativeAngleToTarget(
            Translation2d robot,
            Translation2d target) {
        return target.minus(robot).getAngle();
    }

}
