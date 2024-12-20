package org.team100.frc2024.motion.intake;

import org.team100.frc2024.SensorInterface;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;
import org.team100.lib.motion.servo.LimitedLinearVelocityServo;
import org.team100.lib.motion.servo.ServoFactory;
import org.team100.lib.motor.MotorPhase;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase implements Glassy {
    private static final double kMaxDecel = -10;
    private static final double kMaxAccel = 10;
    private static final double kMaxVelocity = 15;
    private static final double kGearRatio = 9;
    private static final double kWheelDiameterM = 0.05;
    private static final int kCurrentLimit = 20;

    private final SensorInterface m_sensors;

    // this uses PWMSparkMax instead of PWM to get MotorSafety.
    private final PWMSparkMax m_intake;
    private final PWMSparkMax m_centering;
    private final LimitedLinearVelocityServo superRollers;

    // LOGGERS
    private final DoubleLogger m_log_lower;
    private final OptionalDoubleLogger m_log_upper;
    private final DoubleLogger m_log_centering;

    private int count = 0;
    private int currentCount = 0;

    public Intake(LoggerFactory parent, SensorInterface sensors) {
        LoggerFactory child = parent.child(this);
        m_log_lower = child.doubleLogger(Level.TRACE, "lower");
        m_log_upper = child.optionalDoubleLogger(Level.TRACE, "upper");
        m_log_centering = child.doubleLogger(Level.TRACE, "centering");

        m_sensors = sensors;

        switch (Identity.instance) {
            case DISABLED:
                m_intake = new PWMSparkMax(1);
                m_centering = new PWMSparkMax(2);
                superRollers = ServoFactory.limitedNeoVelocityServo(
                        child.child("Super Roller"),
                        5,
                        MotorPhase.FORWARD,
                        kCurrentLimit,
                        kGearRatio,
                        kWheelDiameterM,
                        kMaxVelocity,
                        kMaxAccel,
                        kMaxDecel,
                        Feedforward100.makeNeo(),
                        new PIDConstants(0.0001, 0, 0));
                break;
            case BLANK:
            default:
                m_intake = new PWMSparkMax(1);
                m_centering = new PWMSparkMax(2);
                superRollers = ServoFactory.limitedSimulatedVelocityServo(
                        child.child("Super Roller"),
                        kGearRatio,
                        kWheelDiameterM,
                        kMaxVelocity,
                        kMaxAccel,
                        kMaxDecel);
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
                superRollers.setVelocityM_S(1);
            }
        }

        if (count >= 4) {
            m_intake.set(0);
            m_centering.set(0);
            superRollers.setVelocityM_S(0);
            count = 0;
        }

        currentCount++;
    }

    public void intake() {
        m_centering.set(0.8);
        m_intake.set(-1);
        superRollers.setVelocityM_S(0.8);
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
        superRollers.setVelocityM_S(-0.8);
    }

    public void stop() {
        m_intake.set(0);
        m_centering.set(0);
        superRollers.setVelocityM_S(0);
    }

    @Override
    public void periodic() {
        m_log_lower.log(m_intake::get);
        m_log_upper.log(superRollers::getVelocity);
        m_log_centering.log(m_centering::get);
    }
}
