package org.team100.frc2024.motion.intake;

import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Distance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    //TODO GET THE RIGHT NUMBERS
    private static final double kGearRatio = 1;
    private static final double kWheelDiameter = 1;
    private static final double kMaxVelM_S = 5;
    private static final double kMaxAccelM_S2 = 5;

    private final LimitedVelocityServo<Distance> topRoller;
    private final LimitedVelocityServo<Distance> bottomRoller;

    public IntakeSubsystem(String name1, String name2, int canID1, int canID2) {
        topRoller = ServoFactory.limitedNeoVelocityServo(
                name1,
                canID1,
                false,
                kGearRatio,
                kWheelDiameter,
                kMaxVelM_S,
                kMaxAccelM_S2);

        bottomRoller = ServoFactory.limitedNeoVelocityServo(
                name2,
                canID2,
                false,
                kGearRatio,
                kWheelDiameter,
                kMaxVelM_S,
                kMaxAccelM_S2);
    }

    public void set(double value) {
        topRoller.setVelocity(value);
        bottomRoller.setVelocity(value);
    }

    @Override
    public void periodic() {
        topRoller.periodic();
        bottomRoller.periodic();
    }
}
