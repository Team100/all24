package org.team100.lib.motion.example1d.sled;

import org.team100.lib.motion.example1d.framework.Workstate;

public class SledWorkstate implements Workstate<SledWorkstate> {

    // TODO: make this a full state, not just position
    private final Double m_state;

    public SledWorkstate(Double state) {
        m_state = state;
    }

    @Override
    public SledWorkstate getWorkstate() {
        return this;
    }

    public double getState() {
        return m_state;
    }
}
