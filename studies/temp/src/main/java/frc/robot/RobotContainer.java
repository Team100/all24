
package frc.robot;

import java.io.IOException;

import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LevelPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
  private final OutboardAngularPositionServo m_motor1;
  private final OutboardAngularPositionServo m_motor2;

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

    final TrajectoryVisualization viz = new TrajectoryVisualization(fieldLogger);
    final DriverControl driverControl = new DriverControlProxy(logger, async);

    final Falcon6Motor falconMotor1 = new Falcon6Motor(fieldLogger, 10, MotorPhase.FORWARD, 5, 5, new PIDConstants(0.1),
        Feedforward100.makeShooterFalcon6());
    final Falcon6Motor falconMotor2 = new Falcon6Motor(fieldLogger, 18, MotorPhase.FORWARD, 5, 5, new PIDConstants(0.1),
        Feedforward100.makeShooterFalcon6());

    final RotaryMechanism rotaryMechanism1 = new RotaryMechanism(fieldLogger, falconMotor1,
        new Talon6Encoder(fieldLogger, falconMotor1), 1);
    final RotaryMechanism rotaryMechanism2 = new RotaryMechanism(fieldLogger, falconMotor2,
        new Talon6Encoder(fieldLogger, falconMotor2), 1);

    final CombinedEncoder combinedEncoder1 = new CombinedEncoder(fieldLogger,
        new ProxyRotaryPositionSensor(rotaryMechanism1), rotaryMechanism1);
    final CombinedEncoder combinedEncoder2 = new CombinedEncoder(fieldLogger,
        new ProxyRotaryPositionSensor(rotaryMechanism2), rotaryMechanism2);

    m_motor1 = new OutboardAngularPositionServo(fieldLogger, rotaryMechanism1, combinedEncoder1);
    m_motor2 = new OutboardAngularPositionServo(fieldLogger, rotaryMechanism2, combinedEncoder2);
    Profile100 profile = new TrapezoidProfile100(
        1,
        1,
        0.02);
    m_motor1.setProfile(profile);
    m_motor2.setProfile(profile);

  }

  XboxController controller = new XboxController(0);

  public void teleopPeriodic() {
    boolean input2 = controller.getRightBumper();
    if (input2) {
      m_motor1.setPosition(Math.PI, 1.571);
    } else {
      m_motor1.setPosition(0, 0);
    }
    boolean input3 = controller.getLeftBumper();
    if (input3) {
      m_motor2.setPosition(Math.PI, 1.571);
    } else if (input3 == false) {
      m_motor2.setPosition(0, 0);
    } else {
      m_motor2.setPosition(Math.PI, 1.571);
    }

  }
}
