package org.team100.lib.util;

import java.util.function.DoubleUnaryOperator;

/**
 * Utility for parabolic wave function.
 * 
 * A parabolic wave is the integration of a triangle wave. It looks similar to a
 * sine wave but it's not the same. For one thing, it is strictly positive.
 * 
 */
public class ParabolicWave implements DoubleUnaryOperator {
    /** the amplitude of the triangle wave being integrated */
    private final double a;
    /** period */
    private final double p;

    /** @param a the amplitude of the parabolic wave */
    public ParabolicWave(double a, double p) {
        this.a = 4 * a / p;
        this.p = p;
    }

    @Override
    public double applyAsDouble(double t) {
        t = ((t % p) + p) % p;
        if (t < p / 4) {
            return (2 * a / p) * t * t;
        } else if (t < 3 * p / 4) {
            return a * p / 4 - (2 * a / p) * (t - p / 2) * (t - p / 2);
        } else {
            return (2 * a / p) * (t - p) * (t - p);
        }
    }

}
