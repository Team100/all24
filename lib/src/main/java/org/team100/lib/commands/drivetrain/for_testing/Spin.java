package org.team100.lib.commands.drivetrain.for_testing;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Turn clockwise in place.
 * 
 * Maximum turn rate and acceleration can be configured.
 * 
 * This exists to explore the response of the theta controller.
 */
public class Spin extends Command implements Glassy {
    private static final double kMaxSpeed = 0.5;
    private static final double kAccel = 0.5;

    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicFieldRelativeController m_controller;

    Translation2d m_center;
    double m_initialRotation;
    double m_speedRad_S;
    double m_angleRad;

    public Spin(
            SwerveDriveSubsystem swerve,
            HolonomicFieldRelativeController controller) {
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
        m_speedRad_S += accelRad_S_S * TimedRobot100.LOOP_PERIOD_S;
        if (m_speedRad_S > kMaxSpeed) {
            accelRad_S_S = 0;
            m_speedRad_S = kMaxSpeed;
        }
        m_angleRad += m_speedRad_S * TimedRobot100.LOOP_PERIOD_S;

        Model100 xState = new Model100(m_center.getX(), 0);
        Model100 yState = new Model100(m_center.getY(), 0);
        Model100 rotation = new Model100(
                m_initialRotation + m_angleRad,
                m_speedRad_S);

        SwerveModel reference = new SwerveModel(xState, yState, rotation);

        FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(m_swerve.getState(), reference);
        // force dx and dy to zero, clamp dtheta
        FieldRelativeVelocity clamped = new FieldRelativeVelocity(0, 0,
                MathUtil.clamp(fieldRelativeTarget.theta(), -kMaxSpeed, kMaxSpeed));
        m_swerve.driveInFieldCoords(clamped);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

}
