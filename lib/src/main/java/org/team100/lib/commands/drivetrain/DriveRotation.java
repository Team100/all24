package org.team100.lib.commands.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Rotate in place at the specified speed.
 */
public class DriveRotation extends Command implements Glassy  {
    private final SwerveDriveSubsystem m_robotDrive;
    private final Supplier<Double> m_rotSpeed;

    public DriveRotation(
            SwerveDriveSubsystem robotDrive,
            Supplier<Double> rot) {
        m_robotDrive = robotDrive;
        m_rotSpeed = rot;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        double rot = m_rotSpeed.get();
        if (Math.abs(rot) <= 0.15) {
            rot = 0;
        }

        FieldRelativeVelocity fieldRelative = new FieldRelativeVelocity(0, 0, rot);
        m_robotDrive.driveInFieldCoords(fieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.stop();
    }
}
