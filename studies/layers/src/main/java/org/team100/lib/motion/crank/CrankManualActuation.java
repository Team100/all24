package org.team100.lib.motion.crank;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Supplies actuation based on manual input.
 */
public class CrankManualActuation implements Supplier<CrankActuation> {
    private final DoubleSupplier m_manual;

    public CrankManualActuation(DoubleSupplier manual) {
        m_manual = manual;
    }

    @Override
    public CrankActuation get() {
        return new CrankActuation(m_manual.getAsDouble());
    }
}
