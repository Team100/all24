// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.shooter;

import java.util.Optional;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToShooter extends Command {
  /** Creates a new RotateToShooter. */
  SwerveDriveSubsystem m_drive;
  public RotateToShooter(SwerveDriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Alliance> alliance = DriverStation.getAlliance(); 
    if (!alliance.isPresent()) return;
    PIDController controller = new PIDController(2, 0, 0);
    double measurement = m_drive.getPose().getRotation().getRadians();
    double setpoint = ShooterUtil.getRobotRotationToSpeaker(alliance.get(), 
    m_drive.getPose().getTranslation(), 0).getRadians();

    setpoint = Math100.getMinDistance(measurement, setpoint);
    
    double value = controller.calculate(m_drive.getPose().getRotation().getRadians(), setpoint);
    Twist2d twist = new Twist2d(0, 0, value);
    m_drive.driveInFieldCoords(twist, 0.02);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        Optional<Alliance> alliance = DriverStation.getAlliance(); 
    if (!alliance.isPresent()) {
        Util.warn("no alliance present!");
        return true;
  }
    return Math.abs(ShooterUtil.getRobotRotationToSpeaker(
        alliance.get(), 
    m_drive.getPose().getTranslation(), 0).getRadians() - m_drive.getPose().getRotation().getRadians()) < 0.05;
  }
}
