package org.team100.lib.util;

import java.util.function.BooleanSupplier;

/** Observe a supplier, return true once per positive edge. */
public class Latch implements BooleanSupplier {

    private final BooleanSupplier m_supplier;
    private boolean m_value;
    private boolean m_output;

    public Latch(BooleanSupplier supplier) {
        Arg.nonnull(supplier);
        m_supplier = supplier;
        m_value = supplier.getAsBoolean();
        m_output = false;
    }

    @Override
    public boolean getAsBoolean() {
        boolean result = m_output;
        m_output = false;
        return result;
    }

    public void reset() {
        m_output = false;
    }

    public void periodic() {
        boolean newValue = m_supplier.getAsBoolean();
        if (newValue && !m_value) {
            // positive edge
            m_value = true;
            m_output = true;
        } else if (!newValue && m_value) {
            // negative edge
            m_value = false;
        }
    }

}
