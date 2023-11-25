package org.team100.lib.motion.crank;

/** Represents state in workspace. */
public class Workstate {

    private final Double m_state;

    public Workstate() {
        this(0.0);
    }

    public Workstate(Double state) {
        m_state = state;
    }

    public Workstate getWorkstate() {
        return this;
    }

    // returns position in meters
    // TODO: full state not just position
    public double getState() {
        return m_state;
    }

}
