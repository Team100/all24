package org.team100.commands;

import org.team100.control.Pilot;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** Passes the pilot velocity to the drive. */
public class PilotDrive extends Command {
    private final Pilot m_control;
    private final DriveSubsystem m_drive;

    public PilotDrive(DriveSubsystem drive, Pilot control) {
        m_control = control;
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public String getName() {
        return "Pilot Drive: " + m_drive.getName();
    }

    @Override
    public void execute() {
        m_drive.drive(m_control.driveVelocity());
    }

}
