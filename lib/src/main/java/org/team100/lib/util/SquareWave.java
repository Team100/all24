package org.team100.lib.util;

import java.util.function.DoubleUnaryOperator;

/**
 * Utility for square wave function.
 * 
 * Centered at zero.
 * 
 * https://en.wikipedia.org/wiki/Square_wave
 */
public class SquareWave implements DoubleUnaryOperator {
    /** amplitude */
    private final double a;
    /** period */
    private final double p;

    public SquareWave(double a, double p) {
        this.a = a;
        this.p = p;
    }

    @Override
    public double applyAsDouble(double t) {
        return a * (2 * (2 * Math.floor((t+p/4) / p) - Math.floor(2 * (t+p/4) / p)) + 1);
    }

}
