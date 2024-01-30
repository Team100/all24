// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private CANSparkMax m_motor1;
  private CANSparkMax m_motor2;

  @Override
  public void robotInit() {
    // m_robotContainer = new RobotContainer();
    m_motor1 = new CANSparkMax(3, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(6, MotorType.kBrushless);

    // m_pidController1 = m_motor1.getPIDController();
    // m_pidController2 = m_motor2.getPIDController();

    // m_pidController1.setP(.001);
    // m_pidController2.setP(.001);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_motor1.set(1);
    m_motor2.set(-.7);
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
  public void teleopPeriodic() {}

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
