// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.amp;

import org.team100.frc2024.RobotState100;

import edu.wpi.first.wpilibj2.command.Command;

public class AmpDefault extends Command {
  /** Creates a new AmpDefault. */
  AmpSubsystem m_amp;
  public AmpDefault(AmpSubsystem amp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_amp = amp;
    addRequirements(m_amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_amp.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(RobotState100.getRobotState()){
        case AMPING:
            switch(RobotState100.getAmpState()){
                case UP:
                    // System.out.println("AMPPPPPPPPPPPPPPP");
                    m_amp.setAmpPosition(50);
                    break;
                case DOWN:
                    m_amp.setAmpPosition(0);
                    break;
                case FEED:
                    m_amp.setAmpPosition(5);
                    break;
                default:
            }
            // System.out.println("AHHHHHHHHHHHHHHHHHHHHHHH");

        default:

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
