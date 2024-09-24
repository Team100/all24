package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;

/** Represents motor-shaft encoder, probably some kind of built-in. */
public interface IncrementalBareEncoder extends Glassy {

    /**
     * Should be cached.
     * 
     * Note some rate implementations can be noisy.
     * 
     * If the encoder can't return a valid measurement (e.g. because hardware is not
     * connected), return empty.
     * 
     * @return rad/s
     */
    OptionalDouble getVelocityRad_S();

    /**
     * Should be cached.
     * 
     * If the encoder can't return a valid measurement (e.g. because hardware is not
     * connected), return empty.
     * 
     * @return rad
     */
    OptionalDouble getPositionRad();

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
        return "IncrementalBareEncoder";
    }

    void setEncoderPositionRad(double motorPositionRad);

    /** For logging */
    void periodic();

}
