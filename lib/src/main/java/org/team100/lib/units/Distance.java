package org.team100.lib.units;

/** Marker to keep from mixing up lengths and angles. */
public interface Distance extends Measure {
    default double modulus(double x) {
        return x;
    }
}
