package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.units.Measure100;

/**
 * Used for both angle and length.
 * 
 * @param <T> unit, angle or distance
 */
public interface Encoder100<T extends Measure100> extends Glassy {

    /**
     * Angle or distance depending on parameter.
     * 
     * Distance is meters
     * Angle measure is counterclockwise-positive rad, and accumulates
     * turns; use MathUtil.AngleModulus if you want.
     * 
     * If the encoder can't return a valid measurement (e.g. because hardware is not
     * connected), return empty.
     */
    OptionalDouble getPosition();

    /**
     * Angular or linear velocity depending on parameter.
     * 
     * Distance is m/s.
     * Angle measure is counterclockwise positive, rad/s.
     * Note some rate implementations can be noisy.
     * 
     * If the encoder can't return a valid measurement (e.g. because hardware is not
     * connected), return empty.
     */
    OptionalDouble getRate();

    /**
     * Resets position to zero
     */
    void reset();

    /**
     * Releases the encoder resource, if necessary (e.g. HAL ports).
     */
    void close();

    @Override
    default String getGlassName() {
        return "Encoder100";
    }

}
