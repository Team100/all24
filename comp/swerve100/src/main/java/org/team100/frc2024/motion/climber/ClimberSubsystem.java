package org.team100.frc2024.motion.climber;

import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TODO: add climber to selftest.
 */
public class ClimberSubsystem extends SubsystemBase {
    private static final double kGearRatio = 20.0;
    private static final double kWinchDiameterM = 0.01;
    private static final double kMaxVelM_S = 1;
    private static final double kMaxAccelM_S2 = 1;
    /** Position servo PID kP */
    private static final double kP = 1;
    private final String m_name;

    private final PositionServo<Distance100> s1;
    private final PositionServo<Distance100> s2;

    public ClimberSubsystem( int canID1, int canID2) {
        m_name = Names.name(this);
        s1 = ServoFactory.neoPositionServo(
                m_name + "/Left",
                canID1,
                false,
                kGearRatio,
                kWinchDiameterM,
                kMaxVelM_S,
                kMaxAccelM_S2,
                new PIDController(kP, 0, 0));
        s2 = ServoFactory.neoPositionServo(
                m_name + "/Right",
                canID2,
                true,
                kGearRatio,
                kWinchDiameterM,
                kMaxVelM_S,
                kMaxAccelM_S2,
                new PIDController(kP, 0, 0));
    }

    /** Set velocity in meters per second */
    public void set(double value) {
        s1.setVelocity(value);
        s2.setVelocity(value);
    }

    public void setPosition(double value) {
        s1.setPosition(value);
        s2.setPosition(value);
    }

    @Override
    public void periodic() {
        s1.periodic();
        s2.periodic();
    }
}
