package org.team100.frc2024.motion.intake;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.FeederState100;
import org.team100.frc2024.RobotState100.IntakeState100;
import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.motion.FeederSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class FeederDefault extends Command {
  FeederSubsystem m_feeder;
  SensorInterface m_sensorSubsystem;
  public FeederDefault(FeederSubsystem feeder, SensorInterface sensorSubsystem) {

    m_feeder = feeder;
    m_sensorSubsystem = sensorSubsystem;
    addRequirements(m_feeder);
  }

  @Override
  public void initialize() {
    //
  }

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

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
