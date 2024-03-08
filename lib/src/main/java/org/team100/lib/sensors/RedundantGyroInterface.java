package org.team100.lib.sensors;

import org.team100.lib.dashboard.Glassy;

/**
 * Allows a no-op implementation for robots without a gyro.
 */
public interface RedundantGyroInterface extends Glassy {
    /**
     * Degrees per second, clockwise-positive NED.
     */
    float getRedundantGyroRateNED();

    /**
     * Degrees, [-180,180]
     */
    float getRedundantPitch();

    /**
     * Degrees, [-180,180]
     */
    float getRedundantRoll();

    /**
     * Degrees, [-180,180], clockwise-positive NED.
     */
    float getRedundantYawNED();

    @Override
    default String getGlassName() {
        return "Gyro";
    }
}
