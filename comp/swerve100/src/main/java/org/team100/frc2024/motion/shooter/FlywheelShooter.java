package org.team100.frc2024.motion.shooter;

import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Distance;

public class FlywheelShooter extends Shooter {
    private static final double kGearRatio = 20.0;
    private static final double kWinchDiameterM = 0.01;
    private static final double kMaxVelM_S = 8;
    private static final double kMaxAccelM_S2 = 30;
    private static final double kMaxDecelM_S2 = 30;

    private final LimitedVelocityServo<Distance> leftShooter;
    private final LimitedVelocityServo<Distance> rightShooter;

    public FlywheelShooter(String name1, String name2, int canID1, int canID2) {

        leftShooter = ServoFactory.limitedNeoVelocityServo(
                name1,
                canID1,
                false,
                kGearRatio,
                kWinchDiameterM,
                kMaxVelM_S,
                kMaxAccelM_S2,
                kMaxDecelM_S2);

        rightShooter = ServoFactory.limitedNeoVelocityServo(
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
        leftShooter.setVelocity(value);
        rightShooter.setVelocity(value);
    }

    @Override
    public void periodic() {
        leftShooter.periodic();
        rightShooter.periodic();
    }
}
