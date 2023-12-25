package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Turn clockwise in place.
 * 
 * Maximum turn rate and acceleration can be configured.
 * 
 * This exists to explore the response of the theta controller.
 */
public class Spin extends Command {

    private static final double kDtS = 0.02;
    private static final double kMaxSpeed = 0.5;
    private static final double kAccel = 0.5;

    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicDriveController3 m_controller;

    private Translation2d m_center;
    private double m_initialRotation;
    private double m_speedRad_S;
    private double m_angleRad;

    public Spin(SwerveDriveSubsystem swerve, HolonomicDriveController3 controller) {
        m_swerve = swerve;
        m_controller = controller;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_controller.reset();
        Pose2d currentPose = m_swerve.getPose();
        m_center = currentPose.getTranslation();
        m_initialRotation = currentPose.getRotation().getRadians();
        m_speedRad_S = 0;
        m_angleRad = 0;
    }

    @Override
    public void execute() {
        double accelRad_S_S = kAccel;
        m_speedRad_S += accelRad_S_S * kDtS;
        if (m_speedRad_S > kMaxSpeed) {
            accelRad_S_S = 0;
            m_speedRad_S = kMaxSpeed;
        }
        m_angleRad += m_speedRad_S * kDtS;

        State100 xState = new State100(m_center.getX(), 0, 0);
        State100 yState = new State100(m_center.getY(), 0, 0);
        State100 rotation = new State100(
                m_initialRotation + m_angleRad,
                m_speedRad_S,
                accelRad_S_S);

        SwerveState reference = new SwerveState(xState, yState, rotation);

        Twist2d fieldRelativeTarget = m_controller.calculate(m_swerve.getPose(), reference);
        m_swerve.driveInFieldCoords(fieldRelativeTarget);

    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

}
