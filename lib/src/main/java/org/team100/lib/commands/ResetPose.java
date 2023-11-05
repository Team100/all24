package org.team100.lib.commands;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ResetPose extends Command {
    private final SwerveDriveSubsystem m_robotDrive;
    private final Pose2d m_pose;

    public ResetPose(SwerveDriveSubsystem robotDrive, double x, double y, double theta) {
        m_pose = new Pose2d(new Translation2d(x, y), new Rotation2d(theta));
        m_robotDrive = robotDrive;
    }

    @Override
    public void initialize() {
        m_robotDrive.resetPose(m_pose);
        m_robotDrive.truncate();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
