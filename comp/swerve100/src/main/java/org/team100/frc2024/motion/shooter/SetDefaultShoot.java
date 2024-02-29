// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.shooter;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.ShooterState100;

import edu.wpi.first.wpilibj2.command.Command;

public class SetDefaultShoot extends Command {
  /** Creates a new SetDefaultShoot. */
  ShooterState100 m_state;
  Shooter m_shooter;
  public SetDefaultShoot(Shooter shooter, ShooterState100 state) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_state = state;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.reset();
    RobotState100.changeShooterState(m_state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      RobotState100.changeShooterState(ShooterState100.STOP);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
