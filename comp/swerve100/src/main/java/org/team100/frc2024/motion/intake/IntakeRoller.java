package org.team100.frc2024.motion.intake;

import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TODO: add intake to selftest.
 */
public class IntakeRoller extends Intake {
    
    //TODO GET THE RIGHT NUMBERS
    private static final double kGearRatio = 1;
    private static final double kWheelDiameter = 1;
    private static final double kMaxVelM_S = 5;
    private static final double kMaxAccelM_S2 = 5;
    private static final double kMaxDecelM_S2 = 5;
    private final String m_name;

    private final LimitedVelocityServo<Distance100> topRoller;
    private final LimitedVelocityServo<Distance100> bottomRoller;
    private final SysParam rollerParameter;


    public IntakeRoller(int topCAN, int bottomCAN) {
        m_name = Names.name(this);

        rollerParameter = new SysParam();
        rollerParameter.setkGearRatio(1);
        rollerParameter.setkWheelDiameter(1);
        rollerParameter.setkMaxVelocity(5);
        rollerParameter.setkMaxAccel(5);
        rollerParameter.setkMaxDeccel(5);


        topRoller = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Top Roller",
                topCAN,
                false,
                rollerParameter);

        bottomRoller = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Bottom Roller",
                bottomCAN,
                false,
                rollerParameter);
    }

    @Override
    public void setIntake(double value) {
        topRoller.setVelocity(value);
        bottomRoller.setVelocity(value);
    }

    @Override
    public void periodic() {
        topRoller.periodic();
        bottomRoller.periodic();
    }
}
