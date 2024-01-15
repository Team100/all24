package org.team100.lib.motion.indexer;

import org.team100.lib.motor.turning.NeoTurningMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    private NeoTurningMotor m_motor;
    private final Telemetry t = Telemetry.get();
    private final String m_nameIndexer;
    public IndexerSubsystem(String name, int canID) {
        m_nameIndexer=name;
        m_motor = new NeoTurningMotor(name,canID,true);
    }
    //Rots per second
    public void set(double value) {
        m_motor.setVelocity(value,0);
    }
    @Override
        public void periodic() {
            t.log(Level.DEBUG, m_nameIndexer + " Velocity (RPM)", m_motor.getVelocity());
        }
}
