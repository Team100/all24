package org.team100.lib.commands;

import edu.wpi.first.wpilibj.Timer;

public class MockTimer extends Timer {
    double time = 0;

    @Override
    public void reset() {
        time = 0;
    }

    @Override
    public double get() {
        return time;
    }

}