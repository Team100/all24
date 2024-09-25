package org.team100.frc2024.motion;

import org.team100.frc2024.SensorInterface;
import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Feeder feeds the shooter.
 */
public class FeederSubsystem extends SubsystemBase implements Glassy {
    // this uses PWMSparkMax instead of PWM to get MotorSafety.
    private final PWMSparkMax feedRoller;
    private final SensorInterface m_sensors;

    // LOGGERS
    private final DoubleSupplierLogger2 m_log_speed;

    public FeederSubsystem(SupplierLogger2 parent, SensorInterface sensors) {
        SupplierLogger2 child = parent.child(this);
        m_log_speed = child.doubleLogger(Level.TRACE, "speed");
        switch (Identity.instance) {
            case COMP_BOT:
                feedRoller = new PWMSparkMax(3);
                break;
            case BLANK:
            default:
                feedRoller = new PWMSparkMax(3);
        }
        m_sensors = sensors;
    }

    public void starve() {
        feedRoller.set(-0.2);
    }

    public void feed() {
        feedRoller.set(0.8);
    }

    public void intake() {
        feedRoller.set(0.5);
    }

    public void intakeSmart() {
        if (m_sensors.getFeederSensor()) {
            feedRoller.set(0.5);
        }
    }

    public void outtake() {
        feedRoller.set(-0.1);
    }

    public void stop() {
        feedRoller.set(0);
    }

    @Override
    public void periodic() {
        m_log_speed.log(feedRoller::get);
    }

}
