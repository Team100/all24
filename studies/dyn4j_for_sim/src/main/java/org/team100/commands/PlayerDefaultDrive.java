package org.team100.commands;

import org.team100.control.Pilot;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** The "player" robot is manually controlled. */
public class PlayerDefaultDrive extends Command {
    private static final double kForce = 200;
    private static final double kTorque = 100;

    private final Pilot m_control;
    private final DriveSubsystem m_drive;

    public PlayerDefaultDrive(DriveSubsystem drive, Pilot control) {
        m_control = control;
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public String getName() {
        return "Player Default: " + m_drive.getName();
    }

    @Override
    public void execute() {
        double steer = -m_control.getLeftX(); // axis 0
        double driveX = -m_control.getRightY(); // axis 5
        double driveY = -m_control.getRightX(); // axis 4
        m_drive.apply(driveX * kForce, driveY * kForce, steer * kTorque);
    }

}
