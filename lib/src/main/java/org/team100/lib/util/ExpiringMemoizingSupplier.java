package org.team100.lib.util;

import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Memoize a supplier, and expire the memo.
 * 
 * Use this to rate-limit an expensive supplier.
 * 
 * Cribbed from guava.
 */
public class ExpiringMemoizingSupplier<T> implements Supplier<T> {
    private final Supplier<T> m_delegate;
    private final long m_durationMicroS;
    private final AtomicReference<T> m_value;
    private final AtomicLong m_expirationMicroS;

    /**
     * @param delegate
     * @param durationMicroS zero is allowed, in which case there is no memo.
     */
    public ExpiringMemoizingSupplier(Supplier<T> delegate, long durationMicroS) {
        if (durationMicroS < 0)
            throw new IllegalArgumentException("durationMicroS cannot be negative");
        m_delegate = delegate;
        m_durationMicroS = durationMicroS;
        m_value = new AtomicReference<>();
        // The special value 0 means "not yet initialized".
        m_expirationMicroS = new AtomicLong(0);
    }

    @Override
    public T get() {
        // Zero duration never memoizes.
        if (m_durationMicroS == 0) {
            return m_delegate.get();
        }
        // Double Checked Locking.
        long time = m_expirationMicroS.get();
        long now = RobotController.getFPGATime();
        if (time == 0 || now - time >= 0) {
            synchronized (this) {
                // Recheck for lost race.
                if (time == m_expirationMicroS.get()) {
                    T t = m_delegate.get();
                    m_value.set(t);
                    m_expirationMicroS.set(now + m_durationMicroS);
                    return t;
                }
            }
        }
        return m_value.get();
    }
}
