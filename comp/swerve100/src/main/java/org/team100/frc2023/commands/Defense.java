package org.team100.frc2023.commands;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class Defense extends Command {
    private final SwerveDriveSubsystem m_robotDrive;

    public Defense(SwerveDriveSubsystem swerveDriveSubsystem) {
        m_robotDrive = swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        m_robotDrive.defense();
    }

}
