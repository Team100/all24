package org.team100.util;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

/** Counts positive edges of the supplier. */
public class Counter implements IntSupplier {

    private final BooleanSupplier m_supplier;
    private boolean m_value;
    private int m_count;

    public Counter(BooleanSupplier supplier) {
        Arg.nonnull(supplier);
        m_supplier = supplier;
        m_value = supplier.getAsBoolean();
        m_count = 0;
    }

    @Override
    public int getAsInt() {
        boolean newValue = m_supplier.getAsBoolean();
        if (newValue && !m_value) {
            m_value = true;
            // count positive edge
            m_count++;
        } else if (!newValue && m_value) {
            // ignore negative edge
            m_value = false;
        }
        return m_count;
    }

    public void reset() {
        m_count = 0;
    }

}
