package org.team100.lib.commands.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveBackwards extends Command {
  /** Creates a new DriveBackwards. */
  SwerveDriveSubsystem m_drive;
  double m_length;
  Pose2d m_startingPose;
  boolean isFinished = false;

  public DriveBackwards(SwerveDriveSubsystem drive, double length) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_startingPose = m_drive.getState().pose();
    m_length = length;
  }

  @Override
  public void initialize() {
    m_startingPose = m_drive.getState().pose();
  }

  @Override
  public void execute() {
    if(Math.abs(m_drive.getState().pose().getX() - m_startingPose.getX()) < m_length ){
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-0.1, 0, 0);
        m_drive.setChassisSpeeds(chassisSpeeds, 0.02);
    } else {
        isFinished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    isFinished = false;
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
