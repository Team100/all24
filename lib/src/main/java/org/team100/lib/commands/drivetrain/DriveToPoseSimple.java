package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Given a pose, drive there using only the holonomic controller,
 * no profile, no trajectory.  This doesn't seem to work well to follow
 * a trajectory or profile, don't use it for that, just give it a point
 * you want to go to, and it will go there.  It makes no attempt to
 * impose feasibility constraints or coordinate the axes.
 */
public class DriveToPoseSimple extends Command implements Glassy {
    private final FieldRelativeVelocityLogger m_log_output;
    private final SwerveModel m_goal;
    private final HolonomicFieldRelativeController m_controller;
    private final SwerveDriveSubsystem m_swerve;

    public DriveToPoseSimple(
            LoggerFactory parent,
            Pose2d goal,
            HolonomicFieldRelativeController controller,
            SwerveDriveSubsystem swerve) {
        LoggerFactory child = parent.child(this);
        m_log_output = child.fieldRelativeVelocityLogger(Level.TRACE, "output");
        // goal is motionless at the specified pose.
        m_goal = new SwerveModel(goal);
        m_controller = controller;
        m_swerve = swerve;
    }

    @Override
    public void initialize() {
        // ?
    }

    @Override
    public void execute() {
        SwerveModel measurement = m_swerve.getState();
        FieldRelativeVelocity output = m_controller.calculate(measurement, m_goal);
        m_log_output.log(() -> output);
        m_swerve.driveInFieldCoordsVerbatim(output);
    }

    @Override
    public boolean isFinished() {
        return m_controller.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }
}
