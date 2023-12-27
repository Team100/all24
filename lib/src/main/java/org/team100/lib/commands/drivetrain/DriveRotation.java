package org.team100.lib.commands.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.commands.Command100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Rotate in place at the specified speed.
 */
public class DriveRotation extends Command100 {
    private final SwerveDriveSubsystem m_robotDrive;
    private final Supplier<Double> m_rotSpeed;

    public DriveRotation(SwerveDriveSubsystem robotDrive,
            Supplier<Double> rot) {
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

        Twist2d fieldRelative = new Twist2d(0, 0, rot);
        m_robotDrive.driveInFieldCoords(fieldRelative, dt);
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.stop();
    }
}
