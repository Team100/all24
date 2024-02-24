// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.intake;

import org.team100.frc2024.Robot;
import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.IntakeState100;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeDefault extends Command {
  /** Creates a new IntakeDefault. */
  Intake m_intake;
  public IntakeDefault(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // if(RobotState100.getIntakeState() == IntakeState100.INTAKE){
    //     // System.out.println("WE ARE INTAKING");
    //     m_intake.intake();

    // } else if(RobotState100.getIntakeState() == IntakeState100.OUTTAKE){
    //     // System.out.println("WE ARE OUTTAKING");
    //     m_intake.outtake();

    // }else if(RobotState100.getIntakeState() == IntakeState100.NONE){
    //     // System.out.println("WE ARE NONNNEE");
    //     m_intake.stop();

    // }
    
    switch(RobotState100.getIntakeState()){
        case INTAKE:
            // System.out.println("INTAKINGGGG");
            m_intake.intakeSmart();
            break;
        case OUTTAKE:
            // System.out.println("OUTTTTTTT");
            m_intake.outtake();
            break;
        case STOP:
            // System.out.println("NOOOEEEEE");
            m_intake.stop();
            break;
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
