package org.team100.frc2024.motion.amp;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The feeder is independent from the pivot, so it's a separate subsystem.
 */
public class AmpFeeder extends SubsystemBase implements Glassy {
    private final BareMotor ampDrive;

    public AmpFeeder(LoggerFactory parent) {
        LoggerFactory child = parent.child(this);
        switch (Identity.instance) {
            case DISABLED:
                Feedforward100 ff = Feedforward100.makeNeo();
                PIDConstants pid = new PIDConstants(0, 0, 0);
                ampDrive = new NeoCANSparkMotor(child, 33, MotorPhase.FORWARD, 40, ff, pid);
                break;
            default:
                // For testing and simulation
                // motor speed is rad/s
                ampDrive = new SimulatedBareMotor(child, 600);
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

}
