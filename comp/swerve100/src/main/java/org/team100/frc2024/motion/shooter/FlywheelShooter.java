package org.team100.frc2024.motion.shooter;

import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

public class FlywheelShooter extends Shooter {
    private static final double kGearRatio = 20.0;
    private static final double kWinchDiameterM = 0.01;
    private static final double kMaxVelM_S = 8;
    private static final double kMaxAccelM_S2 = 30;
    private static final double kMaxDecelM_S2 = 30;
    private final String m_name;

    private final LimitedVelocityServo<Distance100> leftShooter;
    private final LimitedVelocityServo<Distance100> rightShooter;
    private final SysParam shooterParam;

    public FlywheelShooter(int canID1, int canID2) {
        m_name = Names.name(this);

        shooterParam = new SysParam();
        shooterParam.setkGearRatio(20);
        shooterParam.setkWheelDiameter(10);
        shooterParam.setkMaxVelocity(8);
        shooterParam.setkMaxAccel(30);
        shooterParam.setkMaxDeccel(30);


        leftShooter = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Left",
                canID1,
                false,
                shooterParam);

        rightShooter = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Right",
                canID2,
                false,
                shooterParam);
        
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
