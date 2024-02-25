// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.amp;


import org.team100.frc2024.Robot;
import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.IntakeState100;

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
    // switch(RobotState100.getRobotState()){
    //     case AMPING:
    //         switch(RobotState100.getAmpState()){
    //             case UP:
    //                 System.out.println("AMP IS GOIIING UPPPPPPPPPP");
    //                 m_amp.setAmpPosition(0.1);
    //                 m_amp.stopFeed();
    //                 break;
    //             case DOWN:
    //                 m_amp.setAmpPosition(-0.1);
    //                 m_amp.stopFeed();
    //                 break;
    //             case NONE:
    //                 m_amp.setAmpPosition(0);
    //                 m_amp.stopFeed();

    //             default:
    //         }
    //         // System.out.println("AHHHHHHHHHHHHHHHHHHHHHHH");

    //     default:
    //         m_amp.setAmpPosition(0);

    //         m_amp.stopFeed();

    // }


        //0.34
    switch(RobotState100.getAmpState()){
                case UP:
                    m_amp.setAmpPosition(0.34);
                    break;
                case DOWN:
                    m_amp.setDutyCycle(-0.3);
                    break;
                case OUTTAKE:
                case NONE:
                    m_amp.setDutyCycle(0);
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
