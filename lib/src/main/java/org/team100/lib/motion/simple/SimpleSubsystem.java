package org.team100.lib.motion.simple;

import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.units.Distance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a simple one-dimensional mechanism. */
public class SimpleSubsystem extends SubsystemBase {

    private final PositionServo<Distance> m_actuator;
    private final SimpleVisualization m_viz;

    // use the factory to instantiate
    SimpleSubsystem(PositionServo<Distance> actuator) {
        m_actuator = actuator;
        m_viz = new SimpleVisualization(this);
    }

    @Override
    public void periodic() {
        m_viz.periodic();
    }

    public double getPosition() {
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

    public void setDutyCycle(double u) {
        m_actuator.setDutyCycle(u);
    }
}
