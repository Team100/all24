package org.team100.frc2024.motion.shooter;

import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

public class DrumShooter extends Shooter {
    private final String m_name;

    private final LimitedVelocityServo<Distance100> topRoller;
    private final LimitedVelocityServo<Distance100> bottomRoller;

    private final SysParam rollerParameter;

    public DrumShooter(int topRollerID, int bottomRollerID) {
        m_name = Names.name(this);


        rollerParameter = SysParam.limitedNeoVelocityServoSystem(
            1, 
            0.1, 
            8, 
            30, 
            30
        );




        topRoller = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Top",

                topRollerID,

                false,
                rollerParameter);

        bottomRoller = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Bottom",
                bottomRollerID,
                true,
               rollerParameter);
    }

    @Override
    public void setVelocity(double value) {
        topRoller.setDutyCycle(value);
        bottomRoller.setDutyCycle(value);
    }

    @Override
    public void periodic() {
        topRoller.periodic();
        bottomRoller.periodic();
    }
}
