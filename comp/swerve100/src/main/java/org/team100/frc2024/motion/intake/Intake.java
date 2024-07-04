package org.team100.frc2024.motion.intake;

import org.team100.frc2024.SensorInterface;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.units.Distance100;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase implements Glassy {
    private final Logger m_logger;
    private final SensorInterface m_sensors;

    // this uses PWMSparkMax instead of PWM to get MotorSafety.
    private final PWMSparkMax m_intake;
    private final PWMSparkMax m_centering;
    private final LimitedVelocityServo<Distance100> superRollers;

    private int count = 0;
    private int currentCount = 0;

    public Intake(Logger parent, SensorInterface sensors) {
        m_logger = parent.child(this);
        m_sensors = sensors;

        SysParam rollerParameter = SysParam.limitedNeoVelocityServoSystem(9, 0.05, 15, 10, -10);

        switch (Identity.instance) {
            case COMP_BOT:
                m_intake = new PWMSparkMax(1);
                m_centering = new PWMSparkMax(2);
                superRollers = ServoFactory.limitedNeoVelocityServo(
                        m_logger.child("Super Roller"),
                        5,
                        MotorPhase.FORWARD,
                        20,
                        rollerParameter,
                        Feedforward100.makeNeo(),
                        new PIDConstants(0.0001, 0, 0));
                break;
            case BLANK:
            default:
                m_intake = new PWMSparkMax(1);
                m_centering = new PWMSparkMax(2);
                superRollers = ServoFactory.limitedSimulatedVelocityServo(
                        m_logger.child("Super Roller"),
                        rollerParameter);
        }
    }

    public void intakeSmart() {
        if (!m_sensors.getFeederSensor()) {
            count++;
        } else {
            if (currentCount >= 0) {
                m_intake.set(-1);
            }
            if (currentCount >= 1) {
                m_intake.set(-1);
                m_centering.set(0.2);
            }
            if (currentCount >= 2) {
                m_intake.set(-1);
                m_centering.set(0.2);
                superRollers.setVelocity(1);
            }
        }

        if (count >= 4) {
            m_intake.set(0);
            m_centering.set(0);
            superRollers.setVelocity(0);
            count = 0;
        }

        currentCount++;
    }

    public void intake() {
        m_centering.set(0.8);
        m_intake.set(-1);
        superRollers.setVelocity(0.8);
    }

    public void resetCurrentCount() {
        currentCount = 0;
    }

    public void runLowerIntake() {
        m_centering.set(0.8);
        m_intake.set(-1);
    }

    public void outtake() {
        m_intake.set(0.8);
        m_centering.set(-0.8);
        superRollers.setVelocity(-0.8);
    }

    public void stop() {
        m_intake.set(0);
        m_centering.set(0);
        superRollers.setVelocity(0);
    }

    @Override
    public void periodic() {
        m_logger.logDouble(Level.DEBUG, "lower", m_intake::get);
        m_logger.log(Level.DEBUG, "upper", superRollers.getVelocity());
        m_logger.logDouble(Level.DEBUG, "centering", m_centering::get);
    }

    @Override
    public String getGlassName() {
        return "Intake";
    }

}
