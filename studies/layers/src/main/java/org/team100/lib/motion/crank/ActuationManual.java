package org.team100.lib.motion.crank;

import java.util.function.DoubleSupplier;

/**
 * Supplies actuation based on manual input.
 */
public class ActuationManual implements Actuations {
    private final DoubleSupplier m_manual;

    public ActuationManual(DoubleSupplier manual) {
        m_manual = manual;
    }

    @Override
    public Actuation get() {
        return new Actuation(m_manual.getAsDouble());
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }
}
