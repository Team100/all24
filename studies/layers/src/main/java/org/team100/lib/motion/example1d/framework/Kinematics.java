package org.team100.lib.motion.example1d.framework;

public interface Kinematics<State, Conf> {
    State forward(Conf x);

    Conf inverse(State x);
}
