package org.team100.lib.motion.crank;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Supplies configuration based on manual input. */
public class CrankManualConfiguration implements Supplier<CrankConfiguration> {
    private final DoubleSupplier m_manual;

    public CrankManualConfiguration(DoubleSupplier manual) {
        m_manual = manual;
    }

    @Override
    public CrankConfiguration get() {
        return new CrankConfiguration(m_manual.getAsDouble());
    }
}