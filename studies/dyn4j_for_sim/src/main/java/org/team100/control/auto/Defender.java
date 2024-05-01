package org.team100.control.auto;

public class Defender implements Autopilot {
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

    @Override
    public void onEnd() {
    }

}
