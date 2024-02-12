// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.commands.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
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
    m_startingPose = m_drive.getPose();
    m_length = length;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DRIVE BACKWARDS");
    m_startingPose = m_drive.getPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_drive.getPose().getX() - m_startingPose.getX()) < m_length ){
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-0.1, 0, 0);
        // m_drive.driveInFieldCoords(new Twist2d(0.1, 0, 0), 0.02);
        m_drive.setChassisSpeeds(chassisSpeeds, 0.02);
    } else {
        isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
