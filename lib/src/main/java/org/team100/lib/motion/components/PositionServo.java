package org.team100.lib.motion.components;

public interface PositionServo<T> {
    void setPosition(T value);

    T getPosition();
}
