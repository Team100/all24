package org.team100.lib.motion.example1d.framework;

public interface SpecFollower<State> {
    State getReference(double time);
}
