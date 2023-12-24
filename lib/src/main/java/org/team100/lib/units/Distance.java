package org.team100.lib.units;

/** Marker to keep from mixing up lengths and angles. */
public class Distance implements Measure100 {
    public static final Distance instance = new Distance();

    private Distance() {
        //
    }

    /** Distance is not periodic, so modulus is identity. */
    @Override
    public double modulus(double x) {
        return x;
    }
}
