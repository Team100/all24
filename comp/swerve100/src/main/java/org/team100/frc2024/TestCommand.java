// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024;

import edu.wpi.first.wpilibj2.command.Command;

public class TestCommand extends Command {
  /** Creates a new TestCommand. */
  String m_name;
  public TestCommand(String name) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_name = name;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(m_name);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("AHHHHHHHHHHHHHH");
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
