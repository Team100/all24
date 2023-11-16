package org.team100.lib.motion.crank;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Supplies configuration based on manual input. */
public class ConfigurationManual implements Supplier<Configuration> {
    private final DoubleSupplier m_manual;

    public ConfigurationManual(DoubleSupplier manual) {
        m_manual = manual;
    }

    @Override
    public Configuration get() {
        return new Configuration(m_manual.getAsDouble());
    }
}