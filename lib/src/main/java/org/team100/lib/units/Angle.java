package org.team100.lib.units;

import edu.wpi.first.math.MathUtil;

/** Marker to keep from mixing up lengths and angles. */
public interface Angle extends Measure {
    default double modulus(double x) {
        return MathUtil.angleModulus(x);
    }
}
