package org.team100.frc2024.motion;

import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase implements Glassy {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final PWM feedRoller;

    public FeederSubsystem() {
        m_name = Names.name(this);
        switch (Identity.instance) {
            case COMP_BOT:
                feedRoller = new PWM(3);
                break;
            case BLANK:
            default:
                feedRoller = new PWM(3);
        }
    }

    public void starve() {
        feedRoller.setSpeed(-0.2);
    }

    public void feed() {
        feedRoller.setSpeed(0.8);
    }

    public void intake() {
        feedRoller.setSpeed(0.5);
    }

    public void outtake() {
        feedRoller.setSpeed(-0.1);
    }

    public void stop() {
        feedRoller.setSpeed(0);
    }

    @Override
    public void periodic() {
        t.log(Level.DEBUG, m_name, "speed", feedRoller.getSpeed());
    }

    @Override
    public String getGlassName() {
        return "Feeder";
    }
}
