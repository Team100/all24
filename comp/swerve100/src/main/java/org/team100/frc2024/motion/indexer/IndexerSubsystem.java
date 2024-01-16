package org.team100.frc2024.motion.indexer;

import org.team100.lib.motor.drive.NeoDriveMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    private NeoDriveMotor m_motor;

    public IndexerSubsystem(String name, int canID) {
        m_motor = new NeoDriveMotor(name, canID, true, 1, 0.1);
    }

    public void set(double value) {
        m_motor.setVelocity(value, 0);
    }
}
