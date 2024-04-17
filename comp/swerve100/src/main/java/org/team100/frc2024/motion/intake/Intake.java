package org.team100.frc2024.motion.intake;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.IntakeState100;
import org.team100.frc2024.SensorInterface;
import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase implements Glassy  {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final SensorInterface m_sensors;

    private final PWM intakePWM;
    private final PWM centeringPWM;
    private final LimitedVelocityServo<Distance100> superRollers;

    private int count = 0;
    private int currentCount = 0;

    public Intake(SensorInterface sensors) {
        m_name = Names.name(this);
        m_sensors = sensors;

        SysParam rollerParameter = SysParam.limitedNeoVelocityServoSystem(9, 0.05, 15, 10, -10);

        switch (Identity.instance) {
            case COMP_BOT:
                intakePWM = new PWM(1);
                centeringPWM = new PWM(2);
                superRollers = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Super Roller",
                        5,
                        true,
                        20,
                        rollerParameter,
                        new FeedforwardConstants(0.122, 0, 0.1, 0.065),
                        new PIDConstants(0.0001, 0, 0));
                break;
            case BLANK:
            default:
                intakePWM = new PWM(1);
                centeringPWM = new PWM(2);
                superRollers = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Super Roller",
                        rollerParameter);
        }
    }

    public void intakeSmart() {
        if (!m_sensors.getFeederSensor()) {
            count++;
        } else {
            if (currentCount >= 0) {
                intakePWM.setSpeed(-1);
            }

            if (currentCount >= 1) {
                intakePWM.setSpeed(-1);
                centeringPWM.setSpeed(0.2);

            }

            if (currentCount >= 2) {
                intakePWM.setSpeed(-1);
                centeringPWM.setSpeed(0.2);
                superRollers.setDutyCycle(1);
            }

        }

        if (count >= 4) {
            intakePWM.setSpeed(0);
            centeringPWM.setSpeed(0);
            superRollers.setVelocity(0);
            count = 0;
            RobotState100.changeIntakeState(IntakeState100.STOP);
        }

        currentCount++;
    }

    public void intake() {
        centeringPWM.setSpeed(0.8);
        intakePWM.setSpeed(-1);
        superRollers.setDutyCycle(0.8);
    }

    public void resetCurrentCount() {
        currentCount = 0;
    }

    public void runLowerIntake() {
        centeringPWM.setSpeed(0.8);
        intakePWM.setSpeed(-1);
    }

    public void outtake() {
        intakePWM.setSpeed(0.8);
        centeringPWM.setSpeed(-0.8);
        superRollers.setVelocity(-0.8);
    }

    public void stop() {
        intakePWM.setSpeed(0);
        centeringPWM.setSpeed(0);
        superRollers.setDutyCycle(0);
    }

    @Override
    public void periodic() {
        t.log(Level.DEBUG, m_name, "lower", intakePWM.getSpeed());
        t.log(Level.DEBUG, m_name, "upper", superRollers.getVelocity());
        t.log(Level.DEBUG, m_name, "centerin", centeringPWM.getSpeed());
    }

    @Override
    public String getGlassName() {
        return "Intake";
    }

}
