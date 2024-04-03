package org.team100.lib.util;

import org.team100.lib.geometry.Vector2d;
import org.team100.lib.persistent_parameter.Parameter;
import org.team100.lib.persistent_parameter.ParameterFactory;

/**
 * Tire/carpet interaction model.
 * 
 * Imposes a maximum acceleration and slips a little below that, proportional to
 * acceleration.
 */
public class Tire {
    private static final double kDefaultSaturation = 1.0;
    private static final double kDefaultSlipAtSaturation = 0.1;

    private final Parameter m_saturation;
    private final Parameter m_slipAtSaturation;

    public Tire(ParameterFactory parameters) {
        m_saturation = parameters.mutable("TireSaturation", kDefaultSaturation);
        m_slipAtSaturation = parameters.mutable("TireSlipAtSaturation", kDefaultSlipAtSaturation);
    }

    public Vector2d actual(Vector2d corner, Vector2d wheel) {
        Vector2d desiredAccel = desiredAccel(corner, wheel);
        double fraction = fraction(desiredAccel);
        double scale = scale(fraction);
        Vector2d scaledAccel = scaledAccel(desiredAccel, scale);
        Vector2d limitedAccel = limit(scaledAccel);
        return corner.plus(limitedAccel);
    }

    Vector2d desiredAccel(Vector2d corner, Vector2d wheel) {
        return wheel.minus(corner);
    }

    // fraction of saturation accel
    double fraction(Vector2d desiredAccel) {
        return desiredAccel.norm() / m_saturation.get();
    }

    // 0.9 at fraction 1.0
    // 0.95 at fraction 0.5
    // 1.0 at fraction 0
    double scale(double fraction) {
        double slip = m_slipAtSaturation.get() * fraction;
        return 1 - slip;
    }

    Vector2d scaledAccel(Vector2d desiredAccel, double scale) {
        return desiredAccel.times(scale);
    }

    Vector2d limit(Vector2d scaledAccel) {
        double norm = scaledAccel.norm();
        if (norm <= m_saturation.get())
            return scaledAccel;
        return scaledAccel.times(1 / norm);
    }
}
