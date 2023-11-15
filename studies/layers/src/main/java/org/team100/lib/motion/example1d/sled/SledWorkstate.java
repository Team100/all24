package org.team100.lib.motion.example1d.sled;

public class SledWorkstate {

    // TODO: make this a full state, not just position
    private final Double m_state;

    public SledWorkstate(Double state) {
        m_state = state;
    }

    public SledWorkstate getWorkstate() {
        return this;
    }

    public double getState() {
        return m_state;
    }
}
