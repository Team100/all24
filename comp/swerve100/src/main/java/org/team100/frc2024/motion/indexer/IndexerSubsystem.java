package org.team100.frc2024.motion.indexer;

import org.team100.lib.controller.State100;
import org.team100.lib.motor.drive.NeoDriveMotor;
import org.team100.lib.profile.Constraints;
import org.team100.lib.profile.TrapezoidProfile100;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    private final NeoDriveMotor m_motor;
    private final TrapezoidProfile100 m_trapezoid;
    private State100 setpoint;

    public IndexerSubsystem(String name, int canID) {
        m_motor = new NeoDriveMotor(name, canID, true, 1, 0.1);
        m_trapezoid = new TrapezoidProfile100(
                new Constraints(4, 10), 0.05);
        setpoint = new State100(0, 0, 0);
    }

    public void set(double value) {
        setpoint = m_trapezoid.calculate(0.02, setpoint, new State100(value, 0, 0));
        m_motor.setVelocity(setpoint.x(), 0);
    }
}
