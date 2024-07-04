package org.team100.lib.commands.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.commands.Command100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Telemetry.Logger;

/**
 * Rotate in place at the specified speed.
 */
public class DriveRotation extends Command100 {
    private final SwerveDriveSubsystem m_robotDrive;
    private final Supplier<Double> m_rotSpeed;

    public DriveRotation(
            Logger parent,
            SwerveDriveSubsystem robotDrive,
            Supplier<Double> rot) {
        super(parent);
        m_robotDrive = robotDrive;
        m_rotSpeed = rot;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute100(double dt) {
        double rot = m_rotSpeed.get();
        if (Math.abs(rot) <= 0.15) {
            rot = 0;
        }

        FieldRelativeVelocity fieldRelative = new FieldRelativeVelocity(0, 0, rot);
        m_robotDrive.driveInFieldCoords(fieldRelative, dt);
    }

    @Override
    public void end100(boolean interrupted) {
        m_robotDrive.stop();
    }
}
