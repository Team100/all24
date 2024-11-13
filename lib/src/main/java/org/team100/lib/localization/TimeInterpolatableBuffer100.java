package org.team100.lib.localization;

import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.SortedMap;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.StringLogger;

import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * Uses an Interpolator to provide interpolated sampling with a history limit.
 * 
 * The buffer is never empty, so get() always returns *something*.
 */
public final class TimeInterpolatableBuffer100<T extends Interpolatable<T>> implements Glassy {
    private final double m_historyS;
    private final NavigableMap<Double, T> m_pastSnapshots = new ConcurrentSkipListMap<>();

    /**
     * We allow concurrent operations on the map, with one exception: when we want
     * two reads to be consistent with each other. For this purpose we use a
     * read-write lock with the semantics inverted: it's the double-read operation
     * that takes the "write" (exclusive) lock, and the write operations take the
     * "read" (non-exclusive) lock.
     */
    private final ReadWriteLock m_lock = new ReentrantReadWriteLock();
    private final StringLogger m_log_bottom;
    private final StringLogger m_log_top;
    private final DoubleLogger m_log_lerpTime;

    public TimeInterpolatableBuffer100(LoggerFactory parent, double historyS, double timeS, T initialValue) {
        LoggerFactory child = parent.child(this);
        m_historyS = historyS;
        // no lock needed in constructor
        m_pastSnapshots.put(timeS, initialValue);
        m_log_bottom = child.stringLogger(Level.TRACE, "bottom");
        m_log_top = child.stringLogger(Level.TRACE, "top");
        m_log_lerpTime = child.doubleLogger(Level.TRACE, "lerptime");
    }

    /**
     * Remove stale entries and add the new one.
     */
    public void put(double timeS, T value) {
        // System.out.println("TimeInterpolatableBuffer.put() " + timeS + " value " +
        // value);
        try {
            // wait for in-progress double-reads
            m_lock.readLock().lock();
            while (!m_pastSnapshots.isEmpty()) {
                Entry<Double, T> oldest = m_pastSnapshots.firstEntry();
                Double oldestTimeS = oldest.getKey();
                double oldestAgeS = timeS - oldestTimeS;
                // if oldest is younger than the history limit, we're done
                if (oldestAgeS < m_historyS)
                    break;
                m_pastSnapshots.remove(oldestTimeS);
            }
            m_pastSnapshots.put(timeS, value);
        } finally {
            m_lock.readLock().unlock();
        }
    }

    /**
     * Remove all entries and add the new one.
     */
    public void reset(double timeS, T value) {
        // System.out.println("TimeInterpolatableBuffer100.reset() " + timeS + " value "
        // + value);
        try {
            // wait for in-progress double-reads
            m_lock.readLock().lock();
            m_pastSnapshots.clear();
            m_pastSnapshots.put(timeS, value);
        } finally {
            m_lock.readLock().unlock();
        }
    }

    /**
     * Sample the buffer at the given time.
     */
    public T get(double timeSeconds) {
        // System.out.println("TimeInterpolatableBuffer100.timeSeconds() " +
        // timeSeconds);
        // Special case for when the requested time is the same as a sample
        T nowEntry = m_pastSnapshots.get(timeSeconds);
        if (nowEntry != null) {
            // System.out.println("now " + nowEntry);
            m_log_lerpTime.log(() -> 0.0);
            return nowEntry;
        }
        Entry<Double, T> topBound = null;
        Entry<Double, T> bottomBound = null;
        try {
            // this pair should be consistent, so prohibit writes briefly.
            m_lock.writeLock().lock();
            topBound = m_pastSnapshots.ceilingEntry(timeSeconds);
            bottomBound = m_pastSnapshots.floorEntry(timeSeconds);
        } finally {
            m_lock.writeLock().unlock();
        }
        // Return the opposite bound if the other is null
        if (topBound == null) {
            String bottomValue = bottomBound.getValue().toString();
            m_log_bottom.log(() -> bottomValue);
            // System.out.println("bottom " + bottomValue);
            m_log_lerpTime.log(() -> 0.0);
            return bottomBound.getValue();
        }
        if (bottomBound == null) {
            String topValue = topBound.getValue().toString();
            m_log_top.log(() -> topValue);
            // System.out.println("top " + topValue);
            m_log_lerpTime.log(() -> 1.0);
            return topBound.getValue();
        }

        // If both bounds exist, interpolate between them.
        // Because T is between [0, 1], we want the ratio of
        // (the difference between the current time and bottom bound) and (the
        // difference between top and bottom bounds).

        String bottomValue = bottomBound.getValue().toString();
        m_log_bottom.log(() -> bottomValue);
        String topValue = topBound.getValue().toString();
        m_log_top.log(() -> topValue);
        double timeSinceBottom = timeSeconds - bottomBound.getKey();
        double timeSpan = topBound.getKey() - bottomBound.getKey();
        double timeFraction = timeSinceBottom / timeSpan;
        m_log_lerpTime.log(() -> timeFraction);
        return bottomBound.getValue().interpolate(topBound.getValue(), timeFraction);
    }

    public SortedMap<Double, T> tailMap(double t, boolean inclusive) {
        return m_pastSnapshots.tailMap(t, inclusive);
    }

    /** True if the timestamp is older than the history window. */
    boolean tooOld(double timestampS) {
        Double newestSeenS = m_pastSnapshots.lastKey();
        double oldestAcceptableS = newestSeenS - m_historyS;
        return timestampS < oldestAcceptableS;
    }

    public Entry<Double, T> lowerEntry(double t) {
        return m_pastSnapshots.lowerEntry(t);
    }

    public Entry<Double, T> ceilingEntry(double arg0) {
        return m_pastSnapshots.ceilingEntry(arg0);
    }
}