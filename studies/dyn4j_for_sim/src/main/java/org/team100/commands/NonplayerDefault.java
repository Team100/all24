package org.team100.commands;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class NonplayerDefault extends Command {
    private final DriveSubsystem m_drive;
    private final Tactics m_tactics;

    public NonplayerDefault(DriveSubsystem drive, CameraSubsystem camera, boolean debug) {
        m_drive = drive;
        m_tactics = new Tactics(drive, camera, debug);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        FieldRelativeVelocity v = m_tactics.apply(new FieldRelativeVelocity(0, 0, 0), true, true, true, false);
        m_drive.drive(v);
    }
}
