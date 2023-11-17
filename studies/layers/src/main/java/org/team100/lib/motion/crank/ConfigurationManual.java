package org.team100.lib.motion.crank;

import java.util.function.DoubleSupplier;

/** Supplies configuration based on manual input. */
public class ConfigurationManual implements Configurations {
    private final DoubleSupplier m_manual;

    public ConfigurationManual(DoubleSupplier manual) {
        m_manual = manual;
    }

    @Override
    public Configuration get() {
        return new Configuration(m_manual.getAsDouble());
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }
}