package org.team100.lib.motion.example1d;

import org.team100.lib.motion.example1d.framework.Actuation;
import org.team100.lib.motion.example1d.framework.Actuator;

/**
 * Example one-dimensional velocity actuator with imperative interface.
 * This could be either a software servo or an offboard servo.
 */
public class VelocityServo1d implements Actuator<Double> {
    Actuation<Double> m_state;

    @Override
    public void set(Actuation<Double> state) {
        // this is just for testing
        m_state = state;
    }

}
