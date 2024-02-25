// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.FieldRelativeDriver;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
//TODO crappy implementation of this just for EPA, will change this - Sanjan
public class ShooterLockCommand extends Command {
  /** Creates a new ShooterLockCommand. */
  ManualWithShooterLock m_driver;
  Supplier<Twist2d> m_twistSupplier;
  SwerveDriveSubsystem m_drive;


  public ShooterLockCommand(ManualWithShooterLock driver, Supplier<Twist2d> twistSupplier, SwerveDriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driver = driver;
    m_twistSupplier = twistSupplier;
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driver.apply(m_drive.getState(), m_twistSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
