package org.team100.lib.motion.example1d.crank;

import java.util.function.DoubleSupplier;

/**
 * Manual configuration control, passing a manual input directly to actuation.
 */
public class CrankManualConfiguration extends CrankConfigurationController {
    private final DoubleSupplier m_manual;

    public CrankManualConfiguration(DoubleSupplier manual) {
        m_manual = manual;
    }

    @Override
    public CrankActuation calculate(CrankConfiguration measurement, CrankConfiguration setpoint) {
        return new CrankActuation(m_manual.getAsDouble());
    }
}
