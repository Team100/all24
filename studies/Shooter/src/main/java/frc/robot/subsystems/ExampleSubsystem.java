// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  private final VictorSP motor1;
  private final VictorSP motor2;

  public ExampleSubsystem() {
    motor1 = new VictorSP(0);
    motor2 = new VictorSP(1);
  }

  public void set(double value) {
    motor1.set(-value);
    motor2.set(value);
    System.out.println("running");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
