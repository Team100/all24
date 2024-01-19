package org.team100.frc2024.motion.intake;

import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Distance;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    //TODO GET THE RIGHT NUMBERS
    private static final double kGearRatio = 1;
    private static final double kWheelDiameter = 1;
    private static final double kMaxVelM_S = 5;
    private static final double kMaxAccelM_S2 = 5;
    private static final double kMaxDecelM_S2 = 5;
    private final String m_name;

    private final LimitedVelocityServo<Distance> topRoller;
    private final LimitedVelocityServo<Distance> bottomRoller;

    public IntakeSubsystem(int topCAN, int bottomCAN) {
        m_name = Names.name(this);

        topRoller = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Top Roller",
                topCAN,
                false,
                kGearRatio,
                kWheelDiameter,
                kMaxVelM_S,
                kMaxAccelM_S2,
                kMaxDecelM_S2);

        bottomRoller = ServoFactory.limitedNeoVelocityServo(
                m_name + "/Bottom Roller",
                bottomCAN,
                false,
                kGearRatio,
                kWheelDiameter,
                kMaxVelM_S,
                kMaxAccelM_S2,
                kMaxDecelM_S2);
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
