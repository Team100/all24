// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion;

import org.team100.frc2024.motion.shooter.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FeedSmart extends Command {
  /** Creates a new FeedSmart. */
  FeederSubsystem m_feeder;
  Shooter m_shooter;
  Timer m_timer;
  boolean atSetpoint = false;
  boolean finished = false;


  public FeedSmart(FeederSubsystem feeder, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_feeder = feeder;
    m_shooter = shooter;
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_shooter.atVelocitySetpoint()){
      m_feeder.feed();
      m_timer.start();
      atSetpoint = true;
    }

    if(atSetpoint){
      if(m_timer.get() > 1){
        finished = true;
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
