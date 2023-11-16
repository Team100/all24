package org.team100.lib.motion.crank;

import java.util.function.Supplier;

public class CrankMeasurement implements Supplier<CrankConfiguration> {
    private final MotorWrapper m_motor;

    public CrankMeasurement(MotorWrapper motor) {
        m_motor = motor;
    }

    @Override
    public CrankConfiguration get() {
        // TODO: full state measurement, or something?
        return new CrankConfiguration(m_motor.getPosition());
    }
    
}
