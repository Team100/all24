package org.team100.frc2024.motion.shooter;

import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Distance;
import org.team100.lib.util.Names;

public class DrumShooter extends Shooter {
    private static final double kGearRatio = 20.0;
    private static final double kWinchDiameterM = 0.01;
    private static final double kMaxVelM_S = 8;
    private static final double kMaxAccelM_S2 = 30;
    private static final double kMaxDecelM_S2 = 30;
    private final String m_name;

    private final LimitedVelocityServo<Distance> topRoller;
    private final LimitedVelocityServo<Distance> bottomRoller;

    public DrumShooter(int canID1, int canID2) {
        m_name = Names.name(this);

        topRoller = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Top",
                canID1,
                false,
                kGearRatio,
                kWinchDiameterM,
                kMaxVelM_S,
                kMaxAccelM_S2,
                kMaxDecelM_S2);

        bottomRoller = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Bottom",
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
