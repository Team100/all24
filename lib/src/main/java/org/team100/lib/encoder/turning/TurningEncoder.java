package org.team100.lib.encoder.turning;

public interface TurningEncoder {
    /**
     * @return Module azimuth angle in radians, counterclockwise-positive.
     *         Accumulates multiple turns; if you need the modulus, use
     *         MathUtil.AngleModulus.
     */
    double getAngle();

    /**
     * Resets angle to zero.
     */
    void reset();

    void close();
}