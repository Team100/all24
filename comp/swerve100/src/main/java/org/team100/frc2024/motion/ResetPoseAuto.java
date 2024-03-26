package org.team100.frc2024.motion;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ResetPoseAuto extends Command {
    private final SwerveDriveSubsystem m_drive;

    public ResetPoseAuto(SwerveDriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Pose2d pose = new Pose2d(new Translation2d(1.38, 5.566847), new Rotation2d(Math.PI));
        m_drive.resetPose(pose);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
