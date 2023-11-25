package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ExampleCommand extends CommandBase {
  private final ExampleSubsystem m_subsystem;
  private final ProfiledPIDController m_controller;
  private final Constraints m_constraints;
  private final CommandXboxController m_driverController;

  public  ExampleCommand(ExampleSubsystem subsystem, CommandXboxController driverController) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
    m_constraints = new Constraints(8, 16);
    m_controller = new ProfiledPIDController(0.3, 0, 0, m_constraints);
    m_driverController = driverController;
  }

  @Override
  public void initialize() {
    System.out.println("asdfasfa");
    m_controller.reset(m_subsystem.get());
  }

  @Override
  public void execute() {
    double leftX = m_driverController.getLeftX() * 16;
    double output = m_controller.calculate(m_subsystem.get(), leftX);
    System.out.printf("%5.3f %5.3f %5.3f\n", leftX, m_subsystem.get(), output);
    m_subsystem.set(output);
    SmartDashboard.putNumber("Output", output);
    SmartDashboard.putNumber("Setpoint", m_controller.getSetpoint().position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
