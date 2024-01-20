package org.team100.lib.units;

/** Marker to keep from mixing up lengths and angles. */
public class Distance100 implements Measure100 {
    public static final Distance100 instance = new Distance100();

    private Distance100() {
        //
    }

    /** Distance is not periodic, so modulus is identity. */
    @Override
    public double modulus(double x) {
        return x;
    }
}
