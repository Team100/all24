package org.team100.lib.commands.drivetrain;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Turn clockwise in place.
 * 
 * Maximum turn rate and acceleration can be configured.
 * 
 * This exists to explore the response of the theta controller.
 */
public class Spin extends Command100 {
    private static final double kMaxSpeed = 0.5;
    private static final double kAccel = 0.5;

    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicDriveController3 m_controller;

    Translation2d m_center;
    double m_initialRotation;
    double m_speedRad_S;
    double m_angleRad;

    public Spin(SwerveDriveSubsystem swerve, HolonomicDriveController3 controller) {
        m_swerve = swerve;
        m_controller = controller;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
        m_controller.reset();
        Pose2d currentPose = m_swerve.getPose();
        m_center = currentPose.getTranslation();
        m_initialRotation = currentPose.getRotation().getRadians();
        m_speedRad_S = 0;
        m_angleRad = 0;
    }

    @Override
    public void execute100(double dt) {
        double accelRad_S_S = kAccel;
        m_speedRad_S += accelRad_S_S * dt;
        if (m_speedRad_S > kMaxSpeed) {
            accelRad_S_S = 0;
            m_speedRad_S = kMaxSpeed;
        }
        m_angleRad += m_speedRad_S * dt;

        State100 xState = new State100(m_center.getX(), 0, 0);
        State100 yState = new State100(m_center.getY(), 0, 0);
        State100 rotation = new State100(
                m_initialRotation + m_angleRad,
                m_speedRad_S,
                accelRad_S_S);

        SwerveState reference = new SwerveState(xState, yState, rotation);

        Twist2d fieldRelativeTarget = m_controller.calculate(m_swerve.getPose(), reference);
        // force dx and dy to zero, clamp dtheta
        Twist2d clamped = new Twist2d(0, 0,
                MathUtil.clamp(fieldRelativeTarget.dtheta, -kMaxSpeed, kMaxSpeed));
        m_swerve.driveInFieldCoords(clamped, dt);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

}
