package org.team100.lib.motion.example1d.crank;

/**
 * Example one-dimensional velocity actuator with imperative interface.
 * This could be either a software servo or an offboard servo.
 */
public class CrankVelocityServo {
    // this is public so the tests can see it.
    public CrankActuation m_state;

    public CrankVelocityServo(CrankActuation initial) {
        if (initial == null)
            throw new IllegalArgumentException("null actuation");
        m_state = initial;
    }

    public void set(CrankActuation state) {
        if (state == null)
            throw new IllegalArgumentException("null actuation");
        // this is just for testing
        m_state = state;
    }

    public CrankActuation get() {
        return m_state;
    }
}
