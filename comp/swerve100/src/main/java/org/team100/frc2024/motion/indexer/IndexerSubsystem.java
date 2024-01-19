package org.team100.frc2024.motion.indexer;

import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Distance;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TODO: add indexer to selftest.
 */
public class IndexerSubsystem extends SubsystemBase {

    // TODO GET THE RIGHT NUMBERS
    private static final double kGearRatio = 1;
    private static final double kWheelDiameter = 1;
    private static final double kMaxVelM_S = 1;
    private static final double kMaxAccelM_S2 = 1;
    private static final double kMaxDecelM_S2 = 1;
    private final String m_name;

    private final LimitedVelocityServo<Distance> driveMotor;

    public IndexerSubsystem(int canID) {
         m_name = Names.name(this);
        driveMotor = ServoFactory.limitedNeoVelocityServo(
                m_name,
                canID,
                false,
                kGearRatio,
                kWheelDiameter,
                kMaxVelM_S,
                kMaxAccelM_S2,
                kMaxDecelM_S2);
    }

    public void set(double value) {
        driveMotor.setVelocity(value);
    }

    @Override
    public void periodic() {
        driveMotor.periodic();
    }
}
