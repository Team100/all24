// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.intake;

import org.team100.frc2024.Robot;
import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.FeederState100;
import org.team100.frc2024.RobotState100.IntakeState100;
import org.team100.frc2024.RobotState100.ShooterState100;
import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.lib.barcode.Sensor;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FeederDefault extends Command {
  /** Creates a new FeederDefault. */
  FeederSubsystem m_feeder;
  SensorInterface m_sensorSubsystem;
  public FeederDefault(FeederSubsystem feeder, SensorInterface sensorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_feeder = feeder;
    m_sensorSubsystem = sensorSubsystem;
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //This stops 15 ms after intake command tells it to stop
    // if(RobotState100.getShooterState() == ShooterState100.FEED){
    //     m_feeder.feed();
    // } else if(RobotState100.getIntakeState() == IntakeState100.INTAKE){
        // m_feeder.feed();
    // } else {
        // m_feeder.stop();
    // }

    // System.out.println("FEEDER DEFAULT IS RUNNING");

    if(RobotState100.getIntakeState() == IntakeState100.INTAKE){
        m_feeder.intake();
    } else if(RobotState100.getFeederState() == FeederState100.FEED) {
        m_feeder.feed();
    } else if(RobotState100.getIntakeState() == IntakeState100.OUTTAKE){
        m_feeder.outtake();

    }else {
        m_feeder.stop();
    }

    // if(RobotState100.getIntakeState() == IntakeState100.INTAKE){
    //     if(m_sensorSubsystem.getFeederSensor()){
    //         m_feeder.stop();
    //     } else {
    //         m_feeder.feed();
    //     }
    // }

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
