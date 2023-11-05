// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  //This value of the goal for the controller. The units are rotations, so -1 to 1. You can feed something to this value over time to change the goal of the PID controller
  private double goal = 30;

  //The ratio of the motor gearing. If the motor spins more than the output of the motor this value should be greater than 1 this case I had a 45:1 motor gearing
  //TODO For now I am just multiplying this by the goal, but there may be a way to give this value to the motor controller, but I could not seem to find a way
  private final int motorGearing = 1;

  //The type of control you want the Spark Max PID controller to do. In this case it is position
  private final ControlType controlType = CANSparkMax.ControlType.kVelocity;

  private Command m_autonomousCommand;
  private CANSparkMax m_motor;
  private SparkMaxPIDController lowerPidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    //The can ID of the motor
    final int CanID = 2;
    m_motor = new CANSparkMax(CanID, MotorType.kBrushless);
    //The restoreFactoryDefaults method can be used to reset the configuration parameters in the SPARK MAX to their factory default state. If no argument is passed, these parameters will not persist between power cycles
      m_motor.restoreFactoryDefaults();
      lowerPidController = m_motor.getPIDController();
      m_encoder = m_motor.getEncoder();
    //The PID values these are just the default values from the example code, they are not tuned to any motor gearing.
    kP = 0.1; 
    kI = 0;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    //Turn on wrapping
    lowerPidController.setPositionPIDWrappingEnabled(true);

    // set PID coefficients
    lowerPidController.setP(kP);
    lowerPidController.setI(kI);
    lowerPidController.setD(kD);
    lowerPidController.setIZone(kIz);
    lowerPidController.setFF(kFF);
    lowerPidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { lowerPidController.setP(p); kP = p; }
    if((i != kI)) { lowerPidController.setI(i); kI = i; }
    if((d != kD)) { lowerPidController.setD(d); kD = d; }
    if((iz != kIz)) { lowerPidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { lowerPidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      lowerPidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; }
      //Give motor goal and controlType
      lowerPidController.setReference(0, controlType, 0, .1, ArbFFUnits.kVoltage);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
