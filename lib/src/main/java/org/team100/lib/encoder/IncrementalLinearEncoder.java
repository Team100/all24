package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;

/** Incremental, i.e. zero is arbitrary, and linear, i.e. measured in meters. */
public interface IncrementalLinearEncoder extends Glassy {
    /**
     * Meters.
     * 
     * If the encoder can't return a valid measurement (e.g. because hardware is not
     * connected), return empty.
     */
    OptionalDouble getPosition();

    /**
     * Meters per second.
     * 
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
        return "IncrementalLinearEncoder";
    }

}
