package org.team100.lib.encoder;

import java.util.OptionalDouble;
import org.team100.lib.dashboard.Glassy;

/**
 * Absolute rotational measurement, as used in, for example, swerve steering,
 * arm angles, shooter angles, etc. Unlike Encoder100<Angle100>, this does not
 * "wind up", it only returns values within [-pi, pi]. This type of sensor
 * cannot be "reset" at runtime.
 */
public interface RotaryPositionSensor extends Glassy {

    /**
     * Counterclockwise-positive rad within [-pi,pi], which is different from the
     * winding encoder behavior.
     * 
     * If the encoder can't return a valid measurement (e.g. because hardware is not
     * connected), return empty.
     */
    OptionalDouble getPositionRad();

    /**
     * Counterclockwise positive, rad/s.
     * Note some rate implementations can be noisy.
     * 
     * If the encoder can't return a valid measurement (e.g. because hardware is not
     * connected), return empty.
     */
    OptionalDouble getRateRad_S();

    /**
     * Releases the encoder resource, if necessary (e.g. HAL ports).
     */
    void close();

    @Override
    default String getGlassName() {
        return "RotaryPositionSensor";
    }

}
