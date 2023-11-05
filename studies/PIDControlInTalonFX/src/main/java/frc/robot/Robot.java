

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final double ticksPerRevolution = 28;
  private final double m_gearRatio = 355/6;
  //The can ID of the motor controller
  private final int canID = 13;
  //I used a TalonFX for this test change if you have a different motor controller use that instead
  private final WPI_TalonSRX m_motor = new WPI_TalonSRX(canID);
  private RobotContainer m_robotContainer; 

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_motor.configFactoryDefault();
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_motor.configNominalOutputForward(0);
		m_motor.configNominalOutputReverse(0);
		m_motor.configPeakOutputForward(1);
		m_motor.configPeakOutputReverse(-1);
    m_motor.configAllowableClosedloopError(0, 0, 30);
    //F is feedforward, none used for this test
    m_motor.config_kF(0, 0);
    // The P value you have for position control changes, but 0.25 was good for me. I also found having a D value helped.
    // The P value for velocity seems to affect how fast the motor will spin, so if the P value is smaller, the motor will go slower by a multiplier of how much lower the P value is. My P value I used was 0.0001. The lower P value allows the motor to not go crazy, this allows for much more control.
    m_motor.config_kP(0, 0.5); // P 0.25
    m_motor.config_kI(0, 0); // P 0
    m_motor.config_kD(0, 0); // P 20
    m_motor.setSensorPhase(true);
    m_motor.setInverted(false);
  }

  @Override
  public void 
  robotPeriodic() {
    System.out.println(m_motor.getSelectedSensorPosition());
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
    double revolutionsPerSec = -1;
        double revsPer100ms = revolutionsPerSec / 10;
        double ticksPer100ms = revsPer100ms * ticksPerRevolution;
        DemandType type = DemandType.ArbitraryFeedForward;
        double Kn = 0.112;
        double Ks = 0.007576;
        double VSat = 11;
        double kFF = (Kn*revolutionsPerSec + Ks*Math.signum(revolutionsPerSec))*m_gearRatio/VSat;
        m_motor.set(ControlMode.Velocity, ticksPer100ms*m_gearRatio, type, kFF);
        SmartDashboard.putNumber("Encoder Value", m_motor.getSelectedSensorPosition()/(m_gearRatio*ticksPerRevolution));
        SmartDashboard.putNumber("Velocity Value", m_motor.getSelectedSensorVelocity()/(ticksPerRevolution*m_gearRatio)*10);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {}
}
