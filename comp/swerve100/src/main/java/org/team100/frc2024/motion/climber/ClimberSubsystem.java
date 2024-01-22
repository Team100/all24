package org.team100.frc2024.motion.climber;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.Positioning;
import org.team100.lib.motion.simple.SimpleVisualization;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TODO: add climber to selftest.
 */
public class ClimberSubsystem extends SubsystemBase implements Positioning {
    private final String m_name;
    private final SysParam m_params;
    private final PositionServo<Distance100> s1;
    private final PositionServo<Distance100> s2;
    private final SimpleVisualization m_viz;

    public ClimberSubsystem(int leftClimberID, int rightClimberID) {
        m_name = Names.name(this);
        m_params = new SysParam(20.0, 0.01, 1, 1, -1);
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
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
                break;
            case BLANK:
            default:
                s1 = ServoFactory.simulatedDistanceServo(
                        m_name + "/Left",
                        m_params,
                        new PIDController(1, 0, 0));
                s2 = ServoFactory.simulatedDistanceServo(
                        m_name + "/Right",
                        m_params,
                        new PIDController(1, 0, 0));
        }
                m_viz = new SimpleVisualization(m_name, this);
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
    public double getPosition() {
        return (s1.getPosition() + s2.getPosition()) / 2;
    }

    @Override
    public void periodic() {
        s1.periodic();
        s2.periodic();
        m_viz.periodic();
    }
}
