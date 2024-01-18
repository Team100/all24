package org.team100.frc2024.motion.indexer;

import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Distance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    // TODO GET THE RIGHT NUMBERS
    private static final double kGearRatio = 1;
    private static final double kWheelDiameter = 1;
    private static final double kMaxVelM_S = 1;
    private static final double kMaxAccelM_S2 = 1;

    private final LimitedVelocityServo<Distance> driveMotor;

    public IndexerSubsystem(String name, int canID) {
        driveMotor = ServoFactory.limitedNeoVelocityServo(
                name,
                canID,
                false,
                kGearRatio,
                kWheelDiameter,
                kMaxVelM_S,
                kMaxAccelM_S2);
    }

    public void set(double value) {
        driveMotor.setVelocity(value);
    }

    @Override
    public void periodic() {
        driveMotor.periodic();
    }
}
