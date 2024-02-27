package org.team100.lib.sensors;

/**
 * Allows a no-op implementation for robots without a gyro.
 */
public interface RedundantGyroInterface {
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
}
