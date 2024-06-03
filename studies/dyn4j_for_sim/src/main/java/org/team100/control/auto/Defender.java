package org.team100.control.auto;

import org.team100.control.Pilot;

public class Defender implements Pilot {
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
