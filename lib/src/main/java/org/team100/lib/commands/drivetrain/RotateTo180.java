package org.team100.lib.commands.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;

//I WILL DELETE THIS AT SOON - SANJAN
public class RotateTo180 extends Command {
    private final SwerveDriveSubsystem m_drive;
    boolean done = false;

    public RotateTo180(SwerveDriveSubsystem drive) {
        m_drive = drive;
    }

    @Override
    public void execute() {
        if (m_drive.getPose().getRotation().getDegrees() < 150 || m_drive.getPose().getRotation().getDegrees() > 210) {
            Twist2d twist = new Twist2d(0, 0, 1);
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
