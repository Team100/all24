package org.team100.control.auto;

import org.team100.control.AutoPilot;

public class Defender extends AutoPilot {
    private boolean enabled = false;

    @Override
    public boolean defend() {
        return enabled;
    }

    @Override
    public void begin() {
        enabled = true;
    }

    @Override
    public void reset() {
        enabled = false;
    }
}
