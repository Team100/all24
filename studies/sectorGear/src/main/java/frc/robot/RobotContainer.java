package frc.robot;

import java.io.IOException;

import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LevelPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motion.mechanism.LimitedRotaryMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.GravityServoInterface;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final GravityServoInterface m_leftMotor;
  private final GravityServoInterface m_rightMotor;
  private final DriverControl m_driverControl;
  private double value;

    public RobotContainer(TimedRobot100 robot) throws IOException {
        final AsyncFactory asyncFactory = new AsyncFactory(robot);
        final Async async = asyncFactory.get();
        final Logging logging = Logging.instance();
        final LevelPoller poller = new LevelPoller(async, logging::setLevel, Level.COMP);
        Util.printf("Using log level %s\n", poller.getLevel().name());
        Util.println("Do not use TRACE in comp, with NT logging, it will overrun");

        final LoggerFactory fieldLogger = logging.fieldLogger;
        final FieldLogger.Log fieldLog = new FieldLogger.Log(fieldLogger);

        final LoggerFactory logger = logging.rootLogger;
        final LoggerFactory sysLog = logger.child("Subsystems");

        final TrajectoryVisualization viz = new TrajectoryVisualization(fieldLogger);
        m_driverControl = new DriverControlProxy(logger, async);
        NeoCANSparkMotor leftMotor = new NeoCANSparkMotor(fieldLogger, 4, MotorPhase.FORWARD, 40,Feedforward100.makeNeoArm(), new PIDConstants(0));
        NeoCANSparkMotor rightMotor = new NeoCANSparkMotor(sysLog, 7, MotorPhase.REVERSE, 40,Feedforward100.makeNeoArm(), new PIDConstants(0));
        RotaryMechanism leftMechanism = new LimitedRotaryMechanism(new SimpleRotaryMechanism(fieldLogger, leftMotor, new CANSparkEncoder(fieldLogger, leftMotor), 105),Math.toRadians(25),Math.toRadians(115));
        RotaryMechanism rightMechanism = new LimitedRotaryMechanism(new SimpleRotaryMechanism(sysLog, rightMotor, new CANSparkEncoder(sysLog, rightMotor), 105),Math.toRadians(25),Math.toRadians(115));
        m_leftMotor = new OutboardGravityServo(new OutboardAngularPositionServo(fieldLogger, leftMechanism, new CombinedEncoder(fieldLogger, new ProxyRotaryPositionSensor(leftMechanism), leftMechanism)), 8.5, Math.PI/2);
        m_rightMotor = new OutboardGravityServo(new OutboardAngularPositionServo(sysLog, rightMechanism, new CombinedEncoder(sysLog, new ProxyRotaryPositionSensor(rightMechanism), rightMechanism)), 8.5, Math.PI/2);
        TrapezoidProfile100 profile = new TrapezoidProfile100(Math.PI, Math.PI, 0.01);
        m_leftMotor.setEncoderPosition(Math.toRadians(114.9));
        m_rightMotor.setEncoderPosition(Math.toRadians(114.9));
        m_leftMotor.reset(); 
        m_rightMotor.reset(); 
        m_leftMotor.setProfile(profile);
        m_rightMotor.setProfile(profile);

  }

  public void teleopPeriodic() {
    m_leftMotor.periodic();
    m_rightMotor.periodic();
    m_leftMotor.setPosition(Math.PI/2);
    m_rightMotor.setPosition(Math.PI/2);
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
