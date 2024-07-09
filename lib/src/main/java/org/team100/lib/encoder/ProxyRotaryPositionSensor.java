package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.motion.RotaryMechanism;

import edu.wpi.first.math.MathUtil;

/**
 * Proxies a RotaryMechanism to produce a RotaryPositionSensor, by taking the angle
 * modulus.
 */
public class ProxyRotaryPositionSensor implements RotaryPositionSensor {
    private final RotaryMechanism m_delegate;

    public ProxyRotaryPositionSensor(RotaryMechanism delegate) {
        m_delegate = delegate;
    }

    @Override
    public OptionalDouble getPositionRad() {
        OptionalDouble pos = m_delegate.getPositionRad();
        if (pos.isEmpty())
            return pos;
        return OptionalDouble.of(MathUtil.angleModulus(pos.getAsDouble()));
    }

    @Override
    public OptionalDouble getRateRad_S() {
        return m_delegate.getVelocityRad_S();
    }

    @Override
    public void close() {
        m_delegate.close();
    }

}
