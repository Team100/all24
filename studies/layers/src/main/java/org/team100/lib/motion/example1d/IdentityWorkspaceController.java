package org.team100.lib.motion.example1d;

import java.util.function.DoubleFunction;

import org.team100.lib.motion.example1d.framework.WorkspaceController;
import org.team100.lib.motion.example1d.framework.Workstate;
import org.team100.lib.profile.MotionState;

/**
 * does nothing but pass the reference as the output, i.e. it's a feedforward
 * control.
 */
public class IdentityWorkspaceController<T extends Workstate<T>> implements WorkspaceController<T> {

    private final DoubleFunction<T> m_maker;

    public IdentityWorkspaceController(DoubleFunction<T> maker) {
        m_maker = maker;
    }

    // TODO: this is wrong, uses only position, use the full state.
    @Override
    public T calculate(T measurement, MotionState reference) {
        return m_maker.apply(reference.getX());
    }

}
