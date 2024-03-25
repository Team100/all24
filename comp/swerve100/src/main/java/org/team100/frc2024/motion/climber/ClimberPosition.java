// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.climber;

import org.team100.lib.controller.State100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberPosition extends Command {
  /** Creates a new ClimberPosition. */
  ClimberSubsystem m_climber;
  PIDController controller = new PIDController(0.1, 0, 0);
  TrapezoidProfile100 rightProfile;
  TrapezoidProfile100 leftProfile;

  State100 right_setpoint;
  State100 left_setpoint;



  public ClimberPosition( ClimberSubsystem climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.zeroClimbers();

    rightProfile = new TrapezoidProfile100(400, 400, 1);
    right_setpoint = new State100(0, 0);
    
    leftProfile = new TrapezoidProfile100(400, 400, 1);
    left_setpoint = new State100(0, 0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightPose = m_climber.getRightPosition();
    double leftPose = m_climber.getLeftPosition();

    State100 goal = new State100(100, 0);


    right_setpoint = rightProfile.calculate(0.02, right_setpoint, goal);

    double right_fB = controller.calculate(rightPose, right_setpoint.x());
    double rightFF = right_setpoint.v();


    left_setpoint = leftProfile.calculate(0.02, left_setpoint, goal);

    double left_fB = controller.calculate(leftPose, left_setpoint.x());
    double left_FF = left_setpoint.v();

    rightFF = rightFF * 0.8;

    left_FF = left_FF * 0.8;

    m_climber.setRight(right_fB + rightFF);
    m_climber.setLeft(left_fB + left_FF);

    // m_climber.setRight(-0.5);

    Telemetry.get().log(Level.DEBUG, "CLIMBERRR", "Right Pose", rightPose);
    Telemetry.get().log(Level.DEBUG, "CLIMBERRR", "Right Setpoint", right_setpoint);
    Telemetry.get().log(Level.DEBUG, "CLIMBERRR", "Goal", goal);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setRight(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
