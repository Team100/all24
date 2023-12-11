package org.team100.lib.util;

import java.util.function.DoubleUnaryOperator;

/**
 * Utility for triangle wave function.
 * 
 * Starts at zero with positive slope.
 * 
 * https://en.wikipedia.org/wiki/Triangle_wave
 */
public class TriangleWave implements DoubleUnaryOperator {
    /** amplitude */
    private final double a;
    /** period */
    private final double p;

    public TriangleWave(double a, double p) {
        this.a = a;
        this.p = p;
    }

    @Override
    public double applyAsDouble(double x) {
        return 4 * a / p * Math.abs((((x - p / 4) % p) + p) % p - p / 2) - a;
    }

}
