package org.team100.lib.commands.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Stop the drivetrain.
 */
public class DriveStop extends Command {
    private final SwerveDriveSubsystem m_robotDrive;

    public DriveStop(SwerveDriveSubsystem robotDrive) {
        m_robotDrive = robotDrive;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        m_robotDrive.stop();
    }
}
