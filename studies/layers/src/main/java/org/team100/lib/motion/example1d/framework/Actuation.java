package org.team100.lib.motion.example1d.framework;

/** Represents the state followed by the actuator, e.g. velocity. */
public interface Actuation<T> {
    public T getActuation();
}
