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

    public DrumShooter(int canID1, int canID2) {
        m_name = Names.name(this);

        rollerParameter = new SysParam();
        rollerParameter.setkGearRatio(1);
        rollerParameter.setkWheelDiameter(0.05);
        rollerParameter.setkMaxVelocity(8);
        rollerParameter.setkMaxAccel(30);
        rollerParameter.setkMaxDeccel(30);



        topRoller = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Top",
                canID1,
                true,
                rollerParameter);

        bottomRoller = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Bottom",
                canID2,
                false,
               rollerParameter);
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
