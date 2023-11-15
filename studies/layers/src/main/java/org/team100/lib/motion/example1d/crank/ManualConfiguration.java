package org.team100.lib.motion.example1d.crank;

import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

import org.team100.lib.motion.example1d.framework.Actuation;
import org.team100.lib.motion.example1d.framework.Configuration;
import org.team100.lib.motion.example1d.framework.ConfigurationController;

/**
 * Manual configuration control, passing a manual input directly to actuation.
 */
public class ManualConfiguration<T extends Configuration<T>, U extends Actuation<U>>
        implements ConfigurationController<T, U> {
    private final DoubleSupplier m_manual;
    private final DoubleFunction<U> m_maker;

    public ManualConfiguration(DoubleSupplier manual, DoubleFunction<U> maker) {
        m_manual = manual;
        m_maker = maker;
    }

    @Override
    public U calculate(T measurement, T setpoint) {
        return m_maker.apply(m_manual.getAsDouble());
    }
}
