package org.team100.lib.motion.example1d.framework;

/**
 * Represents a velocity servo actuator with feedforward.
 * This is implemented directly by CTRE and VEX smart controllers,
 * or it could be implemented in application code for other controllers.
 */
public interface Actuator<T> {
    void set(Actuation<T> state);
}
