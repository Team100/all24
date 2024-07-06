package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.units.Angle100;

import edu.wpi.first.math.MathUtil;

/**
 * Proxies an Encoder100 to produce a RotaryPositionSensor, by taking the angle
 * modulus.
 * 
 * TODO: add some sort of zeroing?
 */
public class ProxyRotaryPositionSensor implements RotaryPositionSensor {
    private final Encoder100<Angle100> m_delegate;

    public ProxyRotaryPositionSensor(Encoder100<Angle100> delegate) {
        m_delegate = delegate;
    }

    @Override
    public OptionalDouble getPositionRad() {
        OptionalDouble pos = m_delegate.getPosition();
        if (pos.isEmpty())
            return pos;
        return OptionalDouble.of(MathUtil.angleModulus(pos.getAsDouble()));
    }

    @Override
    public OptionalDouble getRateRad_S() {
        return m_delegate.getRate();
    }

    @Override
    public void reset() {
        m_delegate.reset();
    }

    @Override
    public void close() {
        m_delegate.close();
    }

}
