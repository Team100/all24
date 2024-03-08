// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion;

import org.team100.frc2024.motion.amp.AmpSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class StowAmpCommand extends Command {
  /** Creates a new StowAmpCommand. */
  AmpSubsystem m_amp;
  boolean finished = false;
  
  public StowAmpCommand(AmpSubsystem amp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_amp = amp;
    addRequirements(m_amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_amp.reset();
    m_amp.setAmpPosition(0.064230);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_amp.setAmpPosition(0.064230);

    if( Math.abs (m_amp.getPositionRad() - 0.064230) < 0.05 ){
        finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
