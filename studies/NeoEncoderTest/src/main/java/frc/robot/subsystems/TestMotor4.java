// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
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
import org.team100.lib.motor.turning.Falcon6TurningMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.units.Angle100;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestMotor4 extends SubsystemBase {
 /** Creates a new ExampleSubsystem. */

 Falcon6TurningMotor motor = new Falcon6TurningMotor( 
"name",
 22,
 MotorPhase.FORWARD,
1.0,
 new PIDConstants(),
 Feedforward100.makeAMSwerveDriveFalcon6()
 );

 private double goal = 0.5;

 public TestMotor4() {

 }



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

  // SmartDashboard.putNumber("goal", goal);
  // SmartDashboard.putNumber("relative goal",
  // sparkMax.getEncoder().getPosition());

  // sparkMax.set(0.1);
  // SmartDashboard.putNumber("MY POSITION IS: ", turningServo.getPosition().getAsDouble());

  motor.setDutyCycle(0.1);;
 }

 @Override
 public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
 }
}
