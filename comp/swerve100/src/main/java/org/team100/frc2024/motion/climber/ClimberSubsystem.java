package org.team100.frc2024.motion.climber;

import org.team100.lib.config.SysParam;
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
    private final String m_name;
    private final SysParam m_params;

    private final PositionServo<Distance100> s1;
    private final PositionServo<Distance100> s2;

    public ClimberSubsystem(int leftClimberID, int rightClimberID) {
        m_name = Names.name(this);
        m_params = new SysParam(20.0, 0.01, 1, 1, -1);
        s1 = ServoFactory.neoDistanceServo(
                m_name + "/Left",
                leftClimberID,
                false,
                m_params,
                new PIDController(1, 0, 0));
        s2 = ServoFactory.neoDistanceServo(
                m_name + "/Right",
                rightClimberID,
                true,
                m_params,
                new PIDController(1, 0, 0));
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
