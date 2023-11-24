package org.team100.lib.motion.components;

public interface VelocityServo<T> {
    void setVelocity(T value);

    T getVelocity();
}
