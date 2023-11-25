
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

public class RobotContainer {
  private final CANSparkMax m_motor;
  private final ExampleSubsystem m_exampleSubsystem;
  private final CommandXboxController m_driverController;

  public RobotContainer() {
    m_driverController = new CommandXboxController(0);
    m_motor = new CANSparkMax(30, MotorType.kBrushless);
    m_exampleSubsystem = new ExampleSubsystem(m_motor);
    m_driverController.b().whileTrue(new ExampleCommand(m_exampleSubsystem, m_driverController));
    m_driverController.leftBumper().onTrue(m_exampleSubsystem.runOnce(m_exampleSubsystem::reset));
  }
  public void periodic() {
    double leftX = m_driverController.getLeftX() * 10;
    SmartDashboard.putNumber("Input", leftX);
  }
}
