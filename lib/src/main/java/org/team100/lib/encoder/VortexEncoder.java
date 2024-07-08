package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.motor.CANSparkMotor;

/**
 * Very simple encoder wrapper to make testing easier.
 * 
 * Uses native units.
 */
public class VortexEncoder implements IncrementalLinearEncoder {
    private final CANSparkMotor m_motor;

    public VortexEncoder(CANSparkMotor motor) {
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
        double rateRPM = m_motor.getRateRPM();
        return OptionalDouble.of(rateRPM);
    }

    @Override
    public void reset() {
        m_motor.resetEncoderPosition();
    }

    @Override
    public void close() {
        //
    }

}
