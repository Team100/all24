package org.team100.lib.sensors;

/** A non-functional implementation, for robots with no gyro. */
public class NullGyro implements Gyro100 {
    @Override
    public float getYawRateNEDDeg_s() {
        return 0;
    }

    @Override
    public float getPitchDeg() {
        return 0;
    }

    @Override
    public float getRollDeg() {
        return 0;
    }

    @Override
    public float getYawNEDDeg() {
        return 0;
    }
}
