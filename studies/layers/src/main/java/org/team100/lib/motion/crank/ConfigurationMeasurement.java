package org.team100.lib.motion.crank;

import java.util.function.Supplier;

public class ConfigurationMeasurement implements Supplier<Configuration> {
    private final MotorWrapper m_motor;

    public ConfigurationMeasurement(MotorWrapper motor) {
        m_motor = motor;
    }

    @Override
    public Configuration get() {
        // TODO: full state measurement, or something?
        return new Configuration(m_motor.getPosition());
    }
    
}
