package org.team100.lib.motor.duty_cycle;

import java.util.OptionalDouble;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.units.Distance100;

/**
 * Very simple encoder wrapper to make testing easier.
 * 
 * Uses native units.
 */
public class VortexEncoder implements Encoder100<Distance100> {
    private final VortexProxy m_motor;

    public VortexEncoder(VortexProxy motor) {
        m_motor = motor;
    }

    @Override
    public OptionalDouble getPosition() {
        return OptionalDouble.of(m_motor.getPosition());
    }

    @Override
    public OptionalDouble getRate() {
        return OptionalDouble.of(m_motor.getVelocity());
    }

    @Override
    public void reset() {
        m_motor.resetPosition();
    }

    @Override
    public void close() {
        //
    }

}
