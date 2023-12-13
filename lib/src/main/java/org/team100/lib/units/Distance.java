package org.team100.lib.units;

/** Marker to keep from mixing up lengths and angles. */
public class Distance implements Measure {
    public static final Distance instance = new Distance();

    private Distance() {
        //
    }

    @Override
    public double modulus(double x) {
        return x;
    }
}
