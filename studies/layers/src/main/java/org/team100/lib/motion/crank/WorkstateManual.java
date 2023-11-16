package org.team100.lib.motion.crank;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Supplies workspace state as manual input.
 */
public class WorkstateManual implements Supplier<Workstate> {
    private final DoubleSupplier m_manual;

    public WorkstateManual(DoubleSupplier manual) {
        m_manual = manual;
    }

    @Override
    public Workstate get() {
        return new Workstate(m_manual.getAsDouble());
    }
}
