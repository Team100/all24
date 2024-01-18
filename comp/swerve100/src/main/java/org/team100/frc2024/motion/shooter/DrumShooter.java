package org.team100.frc2024.motion.shooter;

import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Distance;

public class DrumShooter extends Shooter {
    private static final double kGearRatio = 20.0;
    private static final double kWinchDiameterM = 0.01;
    private static final double kMaxVelM_S = 8;
    private static final double kMaxAccelM_S2 = 30;
    private static final double kMaxDecelM_S2 = 30;

    private final LimitedVelocityServo<Distance> topRoller;
    private final LimitedVelocityServo<Distance> bottomRoller;

    public DrumShooter(String name1, String name2, int canID1, int canID2) {

        topRoller = ServoFactory.limitedNeoVelocityServo(
                name1,
                canID1,
                false,
                kGearRatio,
                kWinchDiameterM,
                kMaxVelM_S,
                kMaxAccelM_S2,
                kMaxDecelM_S2);

        bottomRoller = ServoFactory.limitedNeoVelocityServo(
                name2,
                canID2,
                false,
                kGearRatio,
                kWinchDiameterM,
                kMaxVelM_S,
                kMaxAccelM_S2,
                kMaxDecelM_S2);
    }

    @Override
    public void setVelocity(double value) {
        topRoller.setVelocity(value);
        bottomRoller.setVelocity(value);
    }

    @Override
    public void periodic() {
        topRoller.periodic();
        bottomRoller.periodic();
    }
}
