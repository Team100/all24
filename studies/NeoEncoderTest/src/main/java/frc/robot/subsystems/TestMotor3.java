// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.turning.DutyCycleTurningEncoder;
import org.team100.lib.encoder.turning.EncoderDrive;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.WCPSwerveModule100;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.Profile100;
import org.team100.lib.units.Angle100;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestMotor3 extends SubsystemBase {
 /** Creates a new ExampleSubsystem. */

 PositionServo<Angle100> turningServo;
 SwerveKinodynamics kinodynamics;

 private double goal = 0.5;

 public TestMotor3() {

  kinodynamics = SwerveKinodynamicsFactory.forTest();

  Profile100 profile = kinodynamics.getSteeringProfile();
  PIDConstants turningPidConstants = new PIDConstants(0.5);// 5
  Feedforward100 turningFF = Feedforward100.makeTest();

  turningServo = WCPSwerveModule100.turningServo(
    "Test Motor",
    DutyCycleTurningEncoder.class,
    22,
    0,
    1.0,
    1.0,
    kinodynamics,
    EncoderDrive.INVERSE,
    MotorPhase.FORWARD,
    turningPidConstants,
    turningFF);

 }

 /**
  * Example command factory method.
  *
  * @return a command
  */

 /**
  * An example method querying a boolean state of the subsystem (for example, a
  * digital sensor).
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

  // SmartDashboardputNumber("goal", goal);
  // SmartDashboard.putNumber("relative goal",
  // sparkMax.getEncoder().getPosition());

  // sparkMax.set(0.1);
  double goal = 30;
  SmartDashboard.putNumber("MY POSITION IS: ", turningServo.getPosition().getAsDouble());
  SmartDashboard.putNumber("GOAL ", goal);

  turningServo.setPosition(goal, 0);
 }

 @Override
 public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
 }
}
