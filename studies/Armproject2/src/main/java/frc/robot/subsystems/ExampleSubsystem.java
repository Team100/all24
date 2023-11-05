// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  private final Servo m_servo;
  private final Servo x_servo;
  private final Servo y_servo;
  private final Servo wrist_servo;
  private final Servo claw_servo;
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    m_servo = new Servo(9);
    x_servo = new Servo(8);
    y_servo = new Servo(7);
    wrist_servo = new Servo(5);
    claw_servo = new Servo(6);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_servo.set(m_servo.get()+value);
          /* one-time action goes here */
        });
  }
  public CommandBase MyMethodCommand(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          x_servo.set(x_servo.get()+value);
          /* one-time action goes here */
        });
  }
  public CommandBase YourMethodCommand(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          y_servo.set(y_servo.get()+value);
          /* one-time action goes here */
        });
  }
  public CommandBase HisMethodCommand(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          wrist_servo.set(wrist_servo.get()+value);
          /* one-time action goes here */
        });
  }
  public CommandBase HerMethodCommand(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          claw_servo.set(claw_servo.get()+value);
          /* one-time action goes here */
        });
  }
  public CommandBase TheirMethodCommand(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          x_servo.set(x_servo.get()+value);
          y_servo.set(y_servo.get()+value);
          /* one-time action goes here */
        });
  }
  public CommandBase BestMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          x_servo.set(0.7);
          y_servo.set(0.8);
          /* one-time action goes here */
        });
  }

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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
