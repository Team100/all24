package org.team100.lib.motion.example1d.crank;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Supplies workspace state as manual input.
 */
public class CrankManualWorkstate implements Supplier<CrankWorkstate> {
    private final DoubleSupplier m_manual;

    public CrankManualWorkstate(DoubleSupplier manual) {
        m_manual = manual;
    }

    @Override
    public CrankWorkstate get() {
        return new CrankWorkstate(m_manual.getAsDouble());
    }
}
