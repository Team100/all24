package org.team100.lib.motion.example1d.sled;

import org.team100.lib.motion.example1d.framework.Actuator;

/**
 * Represents a perfectly controllable 1d configuration.
 * 
 * It responds exactly to whatever the actuator wants, kinda like
 * it had zero mass.
 */
public class SimulatedSled {
    /** Actuator attached to the sled */
    private final Actuator<SledActuation> m_actuator;
    /** Current configuration */
    private SledConfiguration m_config;

    public SimulatedSled(Actuator<SledActuation> actuator) {
        m_actuator = actuator;
        m_config = new SledConfiguration(0);
    }

    public void update(double dtSec) {
        m_config = new SledConfiguration(
                m_config.getPositionM() + m_actuator.get().getVelocityM_S() * dtSec);
    }

    public SledConfiguration getConfig() {
        return m_config;
    }

}
