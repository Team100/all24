package org.team100.lib.motion.simple;

import org.team100.lib.motion.components.PositionServoInterface;
import org.team100.lib.units.Distance100;
import org.team100.lib.visualization.SimpleVisualization;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents a simple one-dimensional mechanism.
 */
public class SimpleSubsystem extends SubsystemBase implements Positioning {

    private final PositionServoInterface<Distance100> m_actuator;

    // use the factory to instantiate
    SimpleSubsystem(PositionServoInterface<Distance100> actuator) {
        m_actuator = actuator;
        SimpleVisualization.make("example", this);
    }

    @Override
    public Double getPositionRad() {
        return m_actuator.getPosition();
    }

    public double getVelocity() {
        return m_actuator.getVelocity();
    }

    public void setPosition(double x) {
        m_actuator.setPosition(x);
    }

    public void setVelocity(double v) {
        m_actuator.setVelocity(v);
    }
}
