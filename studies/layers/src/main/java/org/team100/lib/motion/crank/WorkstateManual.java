package org.team100.lib.motion.crank;

import java.util.function.DoubleSupplier;

/**
 * Supplies workspace state as manual input.
 */
public class WorkstateManual implements Workstates {
    private final DoubleSupplier m_manual;

    public WorkstateManual(DoubleSupplier manual) {
        m_manual = manual;
    }

    @Override
    public Workstate get() {
        return new Workstate(m_manual.getAsDouble());
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }
}
