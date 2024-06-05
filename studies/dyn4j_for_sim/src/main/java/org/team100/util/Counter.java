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
        return m_count;
    }

    public void reset() {
        System.out.println("count reset");
        m_count = 0;
    }

    /** Since "get" is not reliably called, this needs to be called periodically. */
    public void periodic() {
        boolean newValue = m_supplier.getAsBoolean();
        if (newValue && !m_value) {
            m_value = true;
            // count positive edge
            System.out.println("count increment");
            m_count++;
        } else if (!newValue && m_value) {
            // ignore negative edge
            m_value = false;
        }
    }

}
