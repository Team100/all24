package org.team100.lib.units;

import edu.wpi.first.math.MathUtil;

/** Marker to keep from mixing up lengths and angles. */
public class Angle implements Measure {
    public static final Angle instance = new Angle();

    private Angle() {
        //
    }

    /** Angle is periodic. */
    @Override
    public double modulus(double x) {
        return MathUtil.angleModulus(x);
    }
}
