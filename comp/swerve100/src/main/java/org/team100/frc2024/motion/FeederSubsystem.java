package org.team100.frc2024.motion;

import org.team100.frc2024.SensorInterface;
import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Feeder feeds the shooter.
 */
public class FeederSubsystem extends SubsystemBase implements Glassy {
    private final Telemetry.Logger t;
    private final String m_name;
    // this uses PWMSparkMax instead of PWM to get MotorSafety.
    private final PWMSparkMax feedRoller;
    private final SensorInterface m_sensors;

    public FeederSubsystem(SensorInterface sensors) {
        m_name = Names.name(this);
        t = Telemetry.get().rootLogger(m_name);
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
        t.logDouble(Level.DEBUG, "speed", feedRoller::get);
    }

    @Override
    public String getGlassName() {
        return "Feeder";
    }
}
