package org.team100.lib.profile;

import java.util.function.BooleanSupplier;

import org.team100.lib.controller.State100;

public class SelectableProfile100 implements Profile100 {
    private final BooleanSupplier m_selector;
    private final Profile100 m_whenTrue;
    private final Profile100 m_whenFalse;

    public SelectableProfile100(BooleanSupplier selector, Profile100 whenTrue, Profile100 whenFalse) {
        m_selector = selector;
        m_whenTrue = whenTrue;
        m_whenFalse = whenFalse;
    }

    @Override
    public State100 calculate(double dt, State100 initial, State100 goal) {
        if (m_selector.getAsBoolean()) {
            return m_whenTrue.calculate(dt, initial, goal);
        }
        return m_whenFalse.calculate(dt, initial, goal);
    }

}
