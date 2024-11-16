package frc.robot;
import java.io.IOException;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
  private final Logging logging = Logging.instance();
  private final Falcon6Motor falconMotor1;
  private final Falcon6Motor falconMotor2;

  public RobotContainer(TimedRobot100 robot) throws IOException {
    LoggerFactory parent = logging.rootLogger;
    falconMotor1 = new Falcon6Motor(parent, 11, MotorPhase.FORWARD, 5, 5, new PIDConstants(0.1),
        Feedforward100.makeShooterFalcon6());
    falconMotor2 = new Falcon6Motor(parent, 18, MotorPhase.FORWARD, 5, 5, new PIDConstants(0.1),
      Feedforward100.makeShooterFalcon6());
  }

  XboxController controller = new XboxController(0);

  public void teleopPeriodic() {
    double input2 = controller.getLeftY();
      falconMotor1.setDutyCycle(input2); 
    double input3 = controller.getRightY();
      falconMotor2.setDutyCycle(-1*input3);
    
    if (controller.getLeftBumper()){
      falconMotor1.setDutyCycle(0.5);
    }
    if (controller.getRightBumper()){
      falconMotor2.setDutyCycle(-0.5);
    } 
    
    if (controller.getLeftTriggerAxis() != 0){
      falconMotor1.setDutyCycle(-0.5);
    }
    if (controller.getRightTriggerAxis() != 0){
      falconMotor2.setDutyCycle(0.5);
    }

  } 
}         