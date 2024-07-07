package org.team100.frc2024.motion.amp;

import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.duty_cycle.NeoProxy;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.units.Distance100;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The feeder is independent from the pivot, so it's a separate subsystem.
 */
public class AmpFeeder extends SubsystemBase implements Glassy {
    private final Logger m_logger;
    private final Motor100<Distance100> ampDrive;

    public AmpFeeder(Logger parent) {
        m_logger = parent.child(this);
        switch (Identity.instance) {
            case COMP_BOT:
                ampDrive = new NeoProxy(m_logger, 33, IdleMode.kBrake, 40);
                break;
            default:
                // For testing and simulation
                // motor speed is rad/s
                ampDrive = new SimulatedMotor<>(m_logger, 600);
        }
    }

    public void outtake() {
        ampDrive.setDutyCycle(-1);
    }

    public void intake() {
        ampDrive.setDutyCycle(1);
    }

    public void stop() {
        ampDrive.stop();
    }

    @Override
    public String getGlassName() {
        return "Amp Feeder";
    }
}
