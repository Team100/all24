package org.team100.lib.commands.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveBackwards extends Command {
    private final SwerveDriveSubsystem m_drive;
    private final double m_length;

    private Pose2d m_startingPose;
    private boolean m_isFinished;

    public DriveBackwards(SwerveDriveSubsystem drive, double length) {
        m_drive = drive;
        m_length = length;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_isFinished = false;
        m_startingPose = m_drive.getState().pose();
    }

    @Override
    public void execute() {
        if (Math.abs(m_drive.getState().pose().getX() - m_startingPose.getX()) < m_length) {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-0.1, 0, 0);
            m_drive.setChassisSpeeds(chassisSpeeds);
        } else {
            m_drive.stop();
            m_isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
