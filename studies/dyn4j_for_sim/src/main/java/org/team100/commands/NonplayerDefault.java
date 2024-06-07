package org.team100.commands;

import org.team100.Debug;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class NonplayerDefault extends Command {
    private final DriveSubsystem m_drive;
    private final Tactics m_tactics;
    private final boolean m_debug;

    public NonplayerDefault(
            DriveSubsystem drive,
            Tactics tactics,
            boolean debug) {
        m_drive = drive;
        m_tactics = tactics;
        m_debug = debug && Debug.enable();
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.println("NonplayerDefault");
        FieldRelativeVelocity v = m_tactics.apply(new FieldRelativeVelocity(0, 0, 0));
        m_drive.drive(v);
    }
}
