package org.team100.lib.commands.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveRotation extends Command {
    private final SwerveDriveSubsystemInterface m_robotDrive;
    private final Supplier<Double> m_rotSpeed;

    public DriveRotation(SwerveDriveSubsystemInterface robotDrive,
            Supplier<Double> rot) {
        m_robotDrive = robotDrive;
        m_rotSpeed = rot;
        if (m_robotDrive.get() != null)
            addRequirements(m_robotDrive.get());
    }

    @Override
    public void execute() {
        double rot = m_rotSpeed.get();
        if (Math.abs(rot) <= 0.15) {
            rot = 0;
        }

        Twist2d fieldRelative = new Twist2d(0, 0, rot);
        m_robotDrive.driveInFieldCoords(fieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.stop();
        Translation2d endTranslation2d = new Translation2d(
                m_robotDrive.getPose().getX(),
                m_robotDrive.getPose().getY());
        Pose2d endPose = new Pose2d(endTranslation2d, new Rotation2d(0));
        m_robotDrive.resetPose(endPose);
    }
}
