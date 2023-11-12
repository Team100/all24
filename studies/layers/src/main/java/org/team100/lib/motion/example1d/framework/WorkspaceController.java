package org.team100.lib.motion.example1d.framework;

public interface WorkspaceController<State, Conf> {
    Configuration<Conf> calculate(Workstate<State> workstate);
}
