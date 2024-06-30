package org.team100.lib.commands.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.wpilibj2.command.Command;

public class RotateTo180 extends Command {
    private final SwerveDriveSubsystem m_drive;
    boolean done = false;

    public RotateTo180(SwerveDriveSubsystem drive) {
        m_drive = drive;
    }

    @Override
    public void execute() {
        if (m_drive.getState().pose().getRotation().getDegrees() < 150 || m_drive.getState().pose().getRotation().getDegrees() > 210) {
            FieldRelativeVelocity twist = new FieldRelativeVelocity(0, 0, 1);
            m_drive.driveInFieldCoords(twist, 0.02);
        } else {
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
