package org.team100.lib.motion.example1d;

import java.util.function.DoubleConsumer;

/**
 * Example one-dimensional velocity actuator with imperative interface.
 * This could be either a software servo or an offboard servo.
 */
public class VelocityServo1d implements DoubleConsumer {
    public double m_velocityM_S;

    @Override
    public void accept(double velocityM_S) {
        // this is just for testing
        m_velocityM_S = velocityM_S;
    }

}
