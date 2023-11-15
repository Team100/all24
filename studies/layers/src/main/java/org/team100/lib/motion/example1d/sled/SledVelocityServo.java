package org.team100.lib.motion.example1d.sled;

/**
 * Example one-dimensional velocity actuator with imperative interface.
 * This could be either a software servo or an offboard servo.
 */
public class SledVelocityServo {
    // this is public so the tests can see it.
    public SledActuation m_state;

    public SledVelocityServo(SledActuation initial) {
        if (initial == null)
            throw new IllegalArgumentException("null actuation");
        m_state = initial;
    }

    public void set(SledActuation state) {
        if (state == null)
            throw new IllegalArgumentException("null actuation");
        // this is just for testing
        m_state = state;
    }

    public SledActuation get() {
        return m_state;
    }
}
