package org.team100.lib.motion.crank;

public class ConfigurationMeasurement implements Configurations {
    private final MotorWrapper m_motor;

    public ConfigurationMeasurement(MotorWrapper motor) {
        m_motor = motor;
    }

    @Override
    public Configuration get() {
        // TODO: full state measurement, or something?
        return new Configuration(m_motor.getPosition());
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }
    
}
