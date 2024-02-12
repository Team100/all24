// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.commands.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;

//I WILL DELETE THIS AT SOON - SANJAN
public class RotateTo180 extends Command {
  /** Creates a new RotateTo180. */
  SwerveDriveSubsystem m_drive;
  boolean done = false;
  public RotateTo180(SwerveDriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_drive.getPose().getRotation().getDegrees() < 150 || m_drive.getPose().getRotation().getDegrees() > 210 ){
        Twist2d twist = new Twist2d(0, 0, 1);
        m_drive.driveInFieldCoords(twist, 0.02);
    } else {
        done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
