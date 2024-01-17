// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.commands.indexer;

import org.team100.lib.commands.Command100;

public class IndexerCommand extends Command100 {
  /** Creates a new IndexerCommand. */
  public IndexerCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize100() {
    
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute100(double dt) {

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
