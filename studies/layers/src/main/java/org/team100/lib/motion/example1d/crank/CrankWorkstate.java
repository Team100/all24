package org.team100.lib.motion.example1d.crank;

public class CrankWorkstate {

    private final Double m_state;

    public CrankWorkstate() {
        this(0.0);
    }

    public CrankWorkstate(Double state) {
        m_state = state;
    }

    public CrankWorkstate getWorkstate() {
        return this;
    }

    // returns position in meters
    // TODO: full state not just position
    public double getState() {
        return m_state;
    }

}
