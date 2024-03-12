// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.intake;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.RobotState100.IntakeState100;

import edu.wpi.first.wpilibj2.command.Command;

public class ChangeIntakeState extends Command {
  /** Creates a new ChangeIntakeState. */
  Intake m_intake;
  SensorInterface m_sensors;
  public ChangeIntakeState(Intake intake, SensorInterface sensors) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_sensors = sensors;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.intakeSmart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_sensors.getFeederSensor();
  }
}
