package org.team100.lib.sensors;

import org.team100.lib.dashboard.Glassy;

/**
 * Allows a no-op implementation for robots without a gyro.
 */
public interface Gyro100 extends Glassy {
    /**
     * Degrees per second, clockwise-positive NED.
     */
    float getYawRateNEDDeg_s();

    /**
     * Degrees, [-180,180]
     */
    float getPitchDeg();

    /**
     * Degrees, [-180,180]
     */
    float getRollDeg();

    /**
     * Degrees, [-180,180], clockwise-positive NED.
     */
    float getYawNEDDeg();

    @Override
    default String getGlassName() {
        return "Gyro";
    }
}
