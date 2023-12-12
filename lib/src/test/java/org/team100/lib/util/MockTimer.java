package org.team100.lib.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * TODO: remove this class, use FPGATimestamp only.  See SimHooks.stepTiming()
 */
public class MockTimer extends Timer {
    public double time = 0;

    @Override
    public void reset() {
        time = 0;
    }

    @Override
    public double get() {
        return time;
    }

}