package org.team100.lib.motion.example1d;

import org.team100.lib.motion.example1d.framework.Actuation;
import org.team100.lib.motion.example1d.framework.Actuator;

/**
 * Example one-dimensional velocity actuator with imperative interface.
 * This could be either a software servo or an offboard servo.
 */
public class VelocityServo1d<T extends Actuation<T>> implements Actuator<T> {
    // this is public so the tests can see it.
    public T m_state;

    public VelocityServo1d(T initial) {
        if (initial == null)
            throw new IllegalArgumentException("null actuation");
        m_state = initial;
    }

    @Override
    public void set(T state) {
        if (state == null)
            throw new IllegalArgumentException("null actuation");
        // this is just for testing
        m_state = state;
    }

    @Override
    public T get() {
        return m_state;
    }
}
