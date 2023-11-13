package org.team100.lib.motion.example1d.framework;

import org.team100.lib.profile.MotionState;

/**
 * A workspace controller consumes setpoints and measurements and produces
 * workstates.
 * 
 * It's a little weird to make the same type, but the idea is that the state
 * includes its own derivatives, so the controller supplies derivatives, i.e.
 * "xdot", in addition to just passing the setpoint through.
 */
public interface WorkspaceController<T extends Workstate<T>> {
    T calculate(T measurement, MotionState reference);
}
