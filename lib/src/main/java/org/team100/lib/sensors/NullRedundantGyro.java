package org.team100.lib.sensors;

/** A non-functional implementation, for robots with no gyro. */
public class NullRedundantGyro implements RedundantGyroInterface {
    @Override
    public float getRedundantGyroRateNED() {
        return 0;
    }

    @Override
    public float getRedundantPitch() {
        return 0;
    }

    @Override
    public float getRedundantRoll() {
        return 0;
    }

    @Override
    public float getRedundantYawNED() {
        return 0;
    }
}
