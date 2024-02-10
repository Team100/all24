// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.commands.drivetrain;


import org.team100.frc2024.motion.drivetrain.manual.ManualWithShooterLock;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveSimple extends Command {
  /** Creates a new DriveSimple. */
  
  ManualWithShooterLock m_driver;
  SwerveDriveSubsystem m_swerve;
  public DriveSimple(SwerveDriveSubsystem swerve, ManualWithShooterLock driver) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driver = driver;
    m_swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SHOOTER LOCK");
    m_driver.reset(m_swerve.getPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Twist2d twist = m_driver.apply(m_swerve.getState(), new Twist2d(0, 0, 0));
    m_swerve.driveInFieldCoords(twist, 0.02);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driver.isAligned();
    // return false;
  }
}
