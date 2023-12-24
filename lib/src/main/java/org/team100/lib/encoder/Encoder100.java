package org.team100.lib.encoder;

import org.team100.lib.units.Measure100;

/**
 * Used for both angle and length.
 * 
 * @param <T> unit, angle or distance
 */
public interface Encoder100<T extends Measure100> {

    /**
     * Angle or distance depending on parameter.
     * 
     * Distance is meters
     * Angle measure is counterclockwise-positive rad, and accumulates
     * turns; use MathUtil.AngleModulus if you want.
     * TODO: use a Measure here.
     */
    double getPosition();

    /**
     * Does not accumulate. For angles, this is measured in RADIANS.
     */
    double getAbsolutePosition();

    /**
     * Angular or linear velocity depending on parameter.
     * 
     * Distance is m/s.
     * Angle measure is counterclockwise positive, rad/s.
     * Note some rate implementations can be noisy.
     * TODO: use a Measure here.
     */
    double getRate();

    /**
     * Resets position to zero
     */
    void reset();

    /**
     * Releases the encoder resource, if necessary (e.g. HAL ports).
     */
    void close();

}
