package org.team100.lib.util;

import java.util.function.Supplier;

/**
 * Cache a supplier forever.
 * 
 * Invalidation is manual -- you should use your periodic() method for that,
 * which should be wired to Subsystem.periodic() somehow.
 * 
 * The idea is for all subsystem.periodics to have no side-effects other than
 * cache flushing, to that the computation cycle gets a each input calculated
 * just once (and cached thereafter).
 */
public class CotemporalCache<T> implements Supplier<T> {
    private final Supplier<T> m_delegate;
    private T m_value;

    public CotemporalCache(Supplier<T> delegate) {
        m_delegate = delegate;
        m_value = null;
    }

    @Override
    public synchronized T get() {
        // synchronized adds ~20ns which seems ok, it's simple.
        if (m_value == null)
            m_value = m_delegate.get();
        // System.out.println("cache get " + m_value);
        return m_value;
    }

    public synchronized void reset() {
        m_value = null;
    }

}
