package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  private CANSparkMax m_motor1;
  private CANSparkMax m_motor2;
    private CANSparkMax m_motor3;
    private TalonFX e;
        private TalonFX b;

  private XboxController m_joystick;
  private SparkMaxPIDController m_pidController1;
  private RelativeEncoder m_encoder1;
  private SparkMaxPIDController m_pidController2;
  // private RelativeEncoder m_encoder2;
  // private SparkMaxPIDController m_pidController3;
  // private RelativeEncoder m_encoder3;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
 private static final double dynamicFrictionFFVolts = 0.065;
 private static final double  staticFrictionFFVolts = 0.1;
 private static final double velocityFFVoltS_Rev = 0.1315;

 /**
  *
  * For voltage compensation, the maximum output voltage.
  */

  @Override
  public void robotInit() {
    e = new TalonFX(1);
    b = new TalonFX(2);
    e.clearStickyFaults();
        b.clearStickyFaults();
    
    // m_motor1 = new CANSparkMax(7, MotorType.kBrushless);
    // m_motor2 = new CANSparkMax(8, MotorType.kBrushless);
    // m_motor3 = new CANSparkMax(1, MotorType.kBrushless);
    // m_motor1.restoreFactoryDefaults();
    // m_motor2.restoreFactoryDefaults();
    //     m_motor3.restoreFactoryDefaults();
    // m_pidController1 = m_motor1.getPIDController();
    // m_encoder1 = m_motor1.getEncoder();
    // m_pidController2 = m_motor2.getPIDController();
    // m_encoder2 = m_motor2.getEncoder();
    // m_pidController3 = m_motor3.getPIDController();
    // m_encoder3 = m_motor3.getEncoder();
    // kP = 0.0001;
    // kI = 0;
    // kD = 0; 
    // kIz = 0; 
    // kFF = 0; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // maxRPM = 5700;
    // m_pidController1.setP(kP);
    // m_pidController1.setI(kI);
    // m_pidController1.setD(kD);
    // m_pidController1.setIZone(kIz);
    // m_pidController1.setFF(kFF);
    // m_pidController1.setOutputRange(kMinOutput, kMaxOutput);
    // m_pidController2.setP(kP);
    // m_pidController2.setI(kI);
    // m_pidController2.setD(kD);
    // m_pidController2.setIZone(kIz);
    // m_pidController2.setFF(kFF);
    // m_pidController2.setOutputRange(kMinOutput, kMaxOutput);
    // m_pidController3.setP(kP);
    // m_pidController3.setI(kI);
    // m_pidController3.setD(kD);
    // m_pidController3.setIZone(kIz);
    // m_pidController3.setFF(kFF);
    // m_pidController3.setOutputRange(kMinOutput, kMaxOutput);
    // m_motor2.setInverted(true);
    // m_joystick = new XboxController(0);
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("Motor 1 Velocity", m_encoder1.getVelocity()/60);
    // SmartDashboard.putNumber("Motor 2 Velocity", m_encoder2.getVelocity()/60);
    // SmartDashboard.putNumber("Motor 3 Velocity", m_encoder3.getVelocity()/60);
  }

  @Override
  public void testPeriodic() {
    // if (m_joystick.getRightBumper()) {
    //   this.set(MathUtil.applyDeadband(m_joystick.getLeftY(), 0.05));
    //   m_motor3.set(m_joystick.getRightY()/3);
    // } else {
    //   this.set(0);
    //   m_motor3.set(0);
    // }
    e.set(0.3);
    b.set(0.3);
  }

  @Override
  public void teleopPeriodic() {
    if (m_joystick.getXButton()) {
              m_motor3.set(0.33);} else {
                m_motor3.set(0);
              }
    if (m_joystick.getRightBumper()) {
      this.setVelocity(0.5*95);
    } else if (m_joystick.getBButton()) {
      this.setVelocity(0.1*95);
      } else if (m_joystick.getYButton()) {
      this.setVelocity(0.15*95);
      } else if (m_joystick.getLeftBumper()) {
        this.setVelocity(-0.15*95);
      } else if (m_joystick.getAButton()) {
        this.setVelocity(95);
    } else {
      double input = -1.0 * m_joystick.getLeftY()*95;
      if (Math.abs(input) < 1) {
        input = 0;
      }
      this.setVelocity(input);
    }
  }

  public void set(double value1,double value2) {
    m_motor1.set(value1);
    m_motor2.set(value2);
  }

  public void set(double value) {
    this.set(value,value);
  }

  // Units are revs per second
  public void setVelocity(double setpoint1, double setpoint2) {
    double setpointRevsPerMin1 = setpoint1*60;
    double setpointRevsPerMin2 = setpoint2*60;
    double FF1 = frictionFF(m_encoder1.getVelocity()/60, setpoint1) + velocityFF(setpoint1);
    double FF2 = frictionFF(m_encoder1.getVelocity()/60, setpoint2) + velocityFF(setpoint2);
    SmartDashboard.putNumber("Motor 1 setpoint", setpoint1);
    m_pidController1.setReference(setpointRevsPerMin1, CANSparkMax.ControlType.kVelocity, 0, FF1, SparkMaxPIDController.ArbFFUnits.kVoltage);
    SmartDashboard.putNumber("Motor 2 setpoint", setpoint2);
    m_pidController2.setReference(setpointRevsPerMin2, CANSparkMax.ControlType.kVelocity, 1, FF2, SparkMaxPIDController.ArbFFUnits.kVoltage);
  }

  public void setVelocity(double value) {
    this.setVelocity(value,value);
  }
  private static double frictionFF(double currentMotorRev_S, double desiredMotorRev_S) {
    double direction = Math.signum(desiredMotorRev_S);
    if (currentMotorRev_S < 0.5) {
        return staticFrictionFFVolts * direction;
    }
    return dynamicFrictionFFVolts * direction;
}
private static double velocityFF(double desiredMotorRev_S) {
    return velocityFFVoltS_Rev * desiredMotorRev_S;
}
}
