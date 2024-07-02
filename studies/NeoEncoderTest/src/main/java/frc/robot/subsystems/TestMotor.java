// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestMotor extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  CANSparkFlex sparkFlex = new CANSparkFlex(13, MotorType.kBrushless);
  SparkAbsoluteEncoder absoluteEncoder = sparkFlex.getAbsoluteEncoder();
  
  private double goal = 0.5;

  public TestMotor() {
    sparkFlex.setInverted(false);
    absoluteEncoder.setInverted(true);
    sparkFlex.getPIDController().setP(2);
    sparkFlex.getPIDController().setFeedbackDevice(absoluteEncoder);
    // sparkFlex.getPIDController().setReference(0.5, ControlType.kPosition);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(absoluteEncoder.getPosition());

    SmartDashboard.putNumber("goal", goal);
    SmartDashboard.putNumber("measurment", absoluteEncoder.getPosition());
    // SmartDashboard.putNumber("goal fr", sparkFlex.getPIDController());
    SmartDashboard.putNumber("relative goal", sparkFlex.getEncoder().getPosition());

    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
