package org.team100.lib.util;

import org.team100.lib.geometry.Vector2d;
import org.team100.lib.persistent_parameter.Parameter;
import org.team100.lib.persistent_parameter.ParameterFactory;

/**
 * Tire/carpet interaction model.
 * 
 * Imposes a maximum acceleration ("saturation") and slips a little below that,
 * proportional to acceleration.
 */
public class Tire {
    // use these in tests
    static final String kSaturationLabel = "TireSaturation (m_s_s)";
    static final String kSlipLabel = "TireSlipAtSaturation (0-1)";
    // this is surely too low
    // TODO: measure for different wheel/floors; Colson on tile will be much lower.
    private static final double kDefaultSaturationM_s_s = 1.0;
    private static final double kDefaultSlipAtSaturation0_1 = 0.1;

    private final Parameter m_saturationM_s_s;
    private final Parameter m_slipAtSaturation0_1;

    public Tire(ParameterFactory parameters) {
        m_saturationM_s_s = parameters.mutable(kSaturationLabel, kDefaultSaturationM_s_s);
        m_slipAtSaturation0_1 = parameters.mutable(kSlipLabel, kDefaultSlipAtSaturation0_1);
    }

    /**
     * Actual corner velocity at the end of the current period after slip and
     * saturation are taken into account.
     * 
     * @param cornerM_s corner velocities at the start of the current period, i.e.
     *                  the previous step.
     * @param wheelM_s  wheel velocities using measurements from wheel sensors: the
     *                  position at the end of the current period minus the position
     *                  at the start.
     * @param dtS       length of the current period
     */
    public Vector2d actual(Vector2d cornerM_s, Vector2d wheelM_s, double dtS) {
        Vector2d desiredAccelM_s_s = desiredAccelM_s_s(cornerM_s, wheelM_s, dtS);
        double fraction = fraction(desiredAccelM_s_s);
        double scale = scale(fraction);
        Vector2d scaledAccelM_s_s = scaledAccelM_s_s(desiredAccelM_s_s, scale);
        Vector2d limitedAccelM_s_s = limit(scaledAccelM_s_s);
        return apply(cornerM_s, limitedAccelM_s_s, dtS);
    }

    //////////////////////////////////

    Vector2d apply(Vector2d speedM_s, Vector2d accelM_s_s, double dtS) {
        return speedM_s.plus(accelM_s_s.times(dtS));
    }

    /**
     * Acceleration implied by the corner speeds (entering the current period) and
     * wheel speeds (exiting the current period).
     */
    Vector2d desiredAccelM_s_s(Vector2d cornerM_s, Vector2d wheelM_s, double dtS) {
        return wheelM_s.minus(cornerM_s).times(1 / dtS);
    }

    /**
     * Fraction of saturation accel.
     */
    double fraction(Vector2d desiredAccelM_s_s) {
        return desiredAccelM_s_s.norm() / m_saturationM_s_s.get();
    }

    /**
     * Resulting slip, expressed as a fraction of the desired acceleration.
     * 
     * Tires slip more when they are pushed harder:
     * 
     * <pre>
     * saturation fraction 0.0 => 100% of desired accel (no slip)
     * saturation fraction 0.5 =>  95% of desired accel
     * saturation fraction 1.0 =>  90% of desired accel (10% slip)
     * </pre>
     */
    double scale(double fraction) {
        double slip = m_slipAtSaturation0_1.get() * Math.min(1.0, Math.max(0.0, fraction));
        return 1 - slip;
    }

    Vector2d scaledAccelM_s_s(Vector2d desiredAccelM_s_s, double scale) {
        return desiredAccelM_s_s.times(scale);
    }

    /** The given acceleration, limited by saturation. */
    Vector2d limit(Vector2d scaledAccelM_s_s) {
        double normM_s_s = scaledAccelM_s_s.norm();
        double saturationM_s_s = m_saturationM_s_s.get();
        if (normM_s_s <= saturationM_s_s)
            return scaledAccelM_s_s;
        return scaledAccelM_s_s.times(saturationM_s_s / normM_s_s);
    }
}
