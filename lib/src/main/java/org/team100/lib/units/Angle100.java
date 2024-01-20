package org.team100.lib.units;

import edu.wpi.first.math.MathUtil;

/** Marker to keep from mixing up lengths and angles. */
public class Angle100 implements Measure100 {
    public static final Angle100 instance = new Angle100();

    private Angle100() {
        //
    }

    /** Angle is periodic. */
    @Override
    public double modulus(double x) {
        return MathUtil.angleModulus(x);
    }
}
