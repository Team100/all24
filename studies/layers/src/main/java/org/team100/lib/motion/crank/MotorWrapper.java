package org.team100.lib.motion.crank;

/**
 * Wraps a motor that provides both actuation and measurement.
 * It provides both closed-loop and direct setters; the caller is expected to
 * avoid using both simultaneously.
 */
public class MotorWrapper {
    // for testing
    public double m_velocity;
    public double m_feedforward;
    public double m_dutyCycle;

    public void setPID(double velocity, double feedforward) {
        m_velocity = velocity;
        m_feedforward = feedforward;
    }

    public void set(double dutyCycle) {
        m_dutyCycle = dutyCycle;
    }

    public double getPosition() {
        return 0.0;
    }

    public double getVelocity() {
        return 0.0;
    }

}
