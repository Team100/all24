package org.team100.lib.motion.indexer;

import org.team100.lib.motor.turning.NeoTurningMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    private NeoTurningMotor m_motor;
    public IndexerSubsystem(String name, int canID) {
        m_motor = new NeoTurningMotor(name,canID,true);
    }
    //Rots per second
    public void set(double value) {
        m_motor.setVelocity(value,0);
    }
}
