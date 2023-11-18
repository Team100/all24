// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.commands;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;

public class Drive01 extends Command {
  /** Creates a new Drive01. */
  SwerveDriveSubsystem m_swerve;
  public Drive01(SwerveDriveSubsystem swerve)  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ChassisSpeeds newChassisSpeeds = new ChassisSpeeds(0.1, 0, 0);
    // m_swerve.setChassisSpeeds(newChassisSpeeds);

    // m_swerve.setVelocity();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // ChassisSpeeds newChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    // m_swerve.setChassisSpeeds(newChassisSpeeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
