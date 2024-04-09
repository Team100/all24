// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.intake;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.IntakeState100;

import edu.wpi.first.wpilibj2.command.Command;

public class RunIntake extends Command {
  /** Creates a new RunIntake. */
  Intake m_intake;
  public RunIntake(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.resetCurrentCount();
    RobotState100.changeIntakeState(IntakeState100.INTAKE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        RobotState100.changeIntakeState(IntakeState100.STOP);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
