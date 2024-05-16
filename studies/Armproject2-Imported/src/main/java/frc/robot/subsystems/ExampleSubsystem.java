// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmAngles;
import frc.robot.ArmKinematics;

public class ExampleSubsystem extends SubsystemBase {
  private final Servo m_servo;
  private final Servo m_upperArm;
  private final Servo m_lowerArm;
  private final Servo m_upper_wrist;
  private final Servo m_claw;
  private final Servo m_lower_wrist;
  private final ArmKinematics m_kinematics;
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    m_servo = new Servo(3);
    m_upperArm = new Servo(5);
    m_lowerArm = new Servo(2);
    m_upper_wrist = new Servo(1);
    m_claw = new Servo(0);
    m_lower_wrist = new Servo(4);
    m_kinematics = new ArmKinematics(0.18, 0.25);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_servo.set(m_servo.get()+value);
          /* one-time action goes here */
        });
  }
  public Command MyMethodCommand(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_upperArm.set(m_upperArm.get()+value);
          /* one-time action goes here */
        });
  }
  public Command YourMethodCommand(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_lowerArm.set(m_lowerArm.get()+value);
          /* one-time action goes here */
        });
  }
  public Command HisMethodCommand(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_upper_wrist.set(m_upper_wrist.get()+value);
          /* one-time action goes here */
        });
  }
  public Command HerMethodCommand(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_claw.set(m_claw.get()+value);
          /* one-time action goes here */
        });
  }
  public Command TheirMethodCommand(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_upperArm.set(m_upperArm.get()+value);
          m_lowerArm.set(m_lowerArm.get()+value);
          /* one-time action goes here */
        });
  }
  public Command BestMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_upperArm.set(0.7);
          m_lowerArm.set(0.8);
          /* one-time action goes here */
        });
  }
  public double getEncoder(Servos servos) {
    switch (servos) {
      case TURRET:
        return m_servo.get();
      
      case LOWER_ARM:
        return m_lowerArm.get();

      case UPPER_ARM:
        return m_upperArm.get();

      case LOWER_WRIST:
        return m_lower_wrist.get();

      case UPPER_WRIST:
        return m_upper_wrist.get();

      case CLAW:
        return m_claw.get();
      
      default:
      throw new UnsupportedOperationException("This servo does not exist");

    }
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

  public Command setCartesian(Translation2d point) {
    ArmAngles angles = m_kinematics.inverse(point);
    double th2 = Math.PI - angles.th2;
    double th1 = Math.PI - angles.th1;
    double theta2 = th2 - th1;
    double newtheta = MathUtil.angleModulus(theta2);
    if (newtheta>0) {
      throw new UnsupportedOperationException("This is outside the range of this arm. Ha Ha.");
    }
    double lowerServoTheta = (9*(angles.th1 + Math.PI/2) / (10*Math.PI)) + 0.05;
    double upperServoTheta = (-1*newtheta / Math.PI) + 0.03;
    return run(
        () -> {
          System.out.println(angles);
          System.out.println(upperServoTheta);
          System.out.println("Upper Arm: " +th2);
          System.out.println("Lower Arm: " +th1);
          m_upperArm.set(upperServoTheta);
          m_lowerArm.set(lowerServoTheta);
          /* one-time action goes here */
        });
  }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  //   m_servo.set(0.5);
  //   x_servo.set(0.5);
  //   y_servo.set(0.5);
  //   wrist_servo.set(0.5);
  //   claw_servo.set(0.5);
  //   last_servo.set(0.5);
  // }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
