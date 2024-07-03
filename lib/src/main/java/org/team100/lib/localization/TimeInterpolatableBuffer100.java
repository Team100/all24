package org.team100.lib.localization;

import java.util.Map.Entry;
import java.util.List;
import java.util.NavigableMap;
import java.util.SortedMap;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * Uses an Interpolator to provide interpolated sampling with a history limit.
 * 
 * The buffer is never empty, so get() always returns *something*.
 */
public final class TimeInterpolatableBuffer100<T extends Interpolatable<T>> {
    private final Telemetry.Logger t;
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

    public TimeInterpolatableBuffer100(double historyS, double timeS, T initialValue) {
        t = Telemetry.get().rootLogger("buffer");
        m_historyS = historyS;
        // no lock needed in constructor
        m_pastSnapshots.put(timeS, initialValue);
    }

    /**
     * Remove stale entries and add the new one.
     */
    public void put(double timeS, T value) {
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
        // Special case for when the requested time is the same as a sample
        T nowEntry = m_pastSnapshots.get(timeSeconds);
        if (nowEntry != null) {
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
            t.log(Level.TRACE, "bottom", bottomBound.getValue().toString());
            return bottomBound.getValue();
        }
        if (bottomBound == null) {
            t.log(Level.TRACE, "top", topBound.getValue().toString());
            return topBound.getValue();
        }

        // If both bounds exist, interpolate between them.
        // Because T is between [0, 1], we want the ratio of
        // (the difference between the current time and bottom bound) and (the
        // difference between top and bottom bounds).

        t.log(Level.TRACE, "bottom", bottomBound.getValue().toString());
        t.log(Level.TRACE, "top", topBound.getValue().toString());
        double timeSinceBottom = timeSeconds - bottomBound.getKey();
        double timeSpan = topBound.getKey() - bottomBound.getKey();
        double timeFraction = timeSinceBottom / timeSpan;
        return bottomBound.getValue().interpolate(topBound.getValue(), timeFraction);
    }

    /**
     * Return the lowerEntry before t. and another floorEntry dt before that.
     * 
     * The first is used as the basis for integration. The second is used to
     * estimate velocity.
     * 
     * Writes are prohibited between the two reads, so that they are consistent.
     * 
     * This might return an empty list (if no entries exist before t) or one item
     * (if one entry exists before t, but there are no entries earlier than dt
     * before that), or two items.
     * 
     * If present, the first item in the list is the lowerEntry, and the second item
     * is the earlierEntry, if present.
     */
    public List<Entry<Double, T>> consistentPair(double t, double dt) {
        try {
            // this pair should be consistent, so prohibit writes briefly.
            m_lock.writeLock().lock();
            Entry<Double, T> lowerEntry = m_pastSnapshots.lowerEntry(t);
            if (lowerEntry == null) {
                // if there's no lower entry, then return nothing.
                return List.of();
            }
            Entry<Double, T> earlierEntry = m_pastSnapshots.floorEntry(lowerEntry.getKey() - dt);
            if (earlierEntry == null) {
                // if there's no earlier entry, return the lower entry alone.
                return List.of(lowerEntry);
            }

            return List.of(lowerEntry, earlierEntry);
        } finally {
            m_lock.writeLock().unlock();
        }
    }

    public SortedMap<Double, T> tailMap(double t, boolean inclusive) {
        return m_pastSnapshots.tailMap(t, inclusive);
    }

    /**
     * The most recent timestamp in seconds. Never throws since the buffer is never
     * empty.
     */
    public double lastKey() {
        return m_pastSnapshots.lastKey();
    }

    /** The most recent entry. */
    public Entry<Double, T> lastEntry() {
        return m_pastSnapshots.lastEntry();
    }

    public Entry<Double, T> lowerEntry(double t) {
        return m_pastSnapshots.lowerEntry(t);
    }

    public Entry<Double, T> ceilingEntry(double arg0) {
        return m_pastSnapshots.ceilingEntry(arg0);
    }
}