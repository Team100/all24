package org.team100.lib.motion.example1d.framework;

/** Represents the state of the robot, e.g. crank angle. */
public interface Configuration<T> {
    T getConfiguration();
}
