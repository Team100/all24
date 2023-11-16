package org.team100.lib.motion.example1d.crank;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Supplies raw manual input as velocities.
 */
public class CrankManualVelocitySupplier1d implements Supplier<CrankWorkstate> {
    private final DoubleSupplier m_manual;

    public CrankManualVelocitySupplier1d(DoubleSupplier manual) {
        m_manual = manual;
    }

    @Override
    public CrankWorkstate get() {
        return new CrankWorkstate(m_manual.getAsDouble());
    }
}
