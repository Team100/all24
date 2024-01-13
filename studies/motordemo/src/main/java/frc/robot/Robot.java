package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;


public class Robot extends TimedRobot {
  private CANSparkMax m_motor1;
  private CANSparkMax m_motor2;
  private XboxController m_joystick;

  @Override
  public void robotInit() {
    m_motor1 = new CANSparkMax(1, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(2, MotorType.kBrushless);
    m_joystick = new XboxController(0);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    m_motor1.set(m_joystick.getRightY());
    m_motor2.set(m_joystick.getLeftY());
  }
}
