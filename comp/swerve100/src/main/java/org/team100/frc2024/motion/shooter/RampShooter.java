// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.shooter;

import java.util.Optional;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class RampShooter extends Command {
  /** Creates a new RampShooter. */
  Shooter m_shooter;
  SwerveDriveSubsystem m_drive;
  public RampShooter(Shooter shooter, SwerveDriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_drive = drive;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.forward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Optional<Alliance> alliance = DriverStation.getAlliance(); 
    // if (!alliance.isPresent()) return;
    // double distance = m_drive.getPose().getTranslation().getDistance(ShooterUtil.getSpeakerTranslation(alliance.get())); 
    // double angle = ShooterUtil.getAngle(distance);
    // m_shooter.setAngle(angle);

    m_shooter.forward();
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
