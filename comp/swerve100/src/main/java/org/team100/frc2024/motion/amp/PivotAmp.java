// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.amp;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotAmp extends Command {
  /** Creates a new PivotAmp. */
  AmpSubsystem m_amp;
  Supplier<Double> m_ampVelocity;
  public PivotAmp(AmpSubsystem amp, Supplier<Double> ampVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_amp = amp;
    m_ampVelocity = ampVelocity;
    addRequirements(m_amp);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_ampVelocity.get() == 0){
        m_amp.setAmpVelocity(m_ampVelocity.get());
    }else{
        m_amp.setAmpPosition(m_amp.getLeftAmpPosition(), m_amp.getRightAmpPosition());
    }

    
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
