package org.team100.lib.sensors;

/** Allows a no-op implementation for robots without a gyro. */
public interface RedundantGyroInterface {
    float getRedundantGyroRateNED();

    float getRedundantGyroZ();

    float getRedundantPitch();

    float getRedundantRoll();

    float getRedundantYawNED();
}
