package org.team100.lib.motion.example1d.framework;

public interface Kinematics<T extends Workstate<T>, U extends Configuration<U>> {
    T forward(U x);

    U inverse(T x);
}
