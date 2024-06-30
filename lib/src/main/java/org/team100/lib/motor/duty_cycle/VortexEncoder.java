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

    /**
     * In 2024 comp season, this was set up as a "distance" encoder that returns
     * *rotations*.
     * 
     * TODO: calibrate in meters.
     */
    @Override
    public OptionalDouble getPosition() {
        double positionRev = m_motor.getPositionRot();
        return OptionalDouble.of(positionRev);
    }

    /**
     * In 2024 comp season, this was set up as a "distance" encoder that returns a
     * rate in RPM.
     * 
     * TODO: calibrate in meters/sec.
     */
    @Override
    public OptionalDouble getRate() {
        double rateRPM = m_motor.getVelocityRPM();
        return OptionalDouble.of(rateRPM);
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
