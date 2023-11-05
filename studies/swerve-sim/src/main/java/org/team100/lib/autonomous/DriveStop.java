package org.team100.lib.autonomous;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

/** Stops the drivetrain. */
public class DriveStop extends Command {
    private final Drivetrain m_robotDrive;

    public DriveStop(Drivetrain robotDrive) {
        m_robotDrive = robotDrive;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        m_robotDrive.stop();
    }
}