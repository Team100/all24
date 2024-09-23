package org.team100.lib.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Cache a supplier until reset().
 * 
 * The easiest way to wire up reset() is to let Robot.robotPeriodic() call
 * resetAll(). But it's also ok to call reset() on demand, if you have a reason
 * (e.g. resetting a pose, and then wanting to do some more calculation with the
 * just-reset version).
 */
public class CotemporalCache<T> implements Supplier<T> {
    private static final List<Runnable> resetters = new ArrayList<>();

    private final Supplier<T> m_delegate;
    private T m_value;

    public CotemporalCache(Supplier<T> delegate) {
        m_delegate = delegate;
        m_value = null;
        resetters.add(this::reset);
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

    /** This should be run in Robot.robotPeriodic(). */
    public static void resetAll() {
        for (Runnable r : resetters) {
            r.run();
        }
    }
}
