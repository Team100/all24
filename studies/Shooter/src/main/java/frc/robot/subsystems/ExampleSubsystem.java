// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  private final CANSparkMax motor1;
  private final CANSparkMax motor2;

  public ExampleSubsystem() {
    motor1 = new CANSparkMax(1, MotorType.kBrushed);
    motor2 = new CANSparkMax(2, MotorType.kBrushed);
  }

  public void set(double value) {
    motor1.set(-value);
    motor2.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
