package org.team100.frc2024.motion.shooter;

import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

public class FlywheelShooter extends Shooter {
    private final String m_name;

    private final LimitedVelocityServo<Distance100> leftShooter;
    private final LimitedVelocityServo<Distance100> rightShooter;
    private final SysParam shooterParam;

    public FlywheelShooter(int leftShooterID, int rightShooterID) {
        m_name = Names.name(this);



        shooterParam = SysParam.limitedNeoVelocityServoSystem(
            20, 
            10, 
            8, 
            30, 
            30
        );
      


        leftShooter = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Left",
                leftShooterID,
                false,
                shooterParam);

        rightShooter = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Right",
                rightShooterID,
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
