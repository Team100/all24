package org.team100.lib.localization;

import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.SortedMap;
import java.util.concurrent.ConcurrentSkipListMap;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * Uses an Interpolator to provide interpolated sampling with a history limit.
 * 
 * The buffer is never empty, so get() always returns *something*.
 */
public final class TimeInterpolatableBuffer100<T extends Interpolatable<T>> {
    private static final Telemetry t = Telemetry.get();

    private final double m_historyS;
    // @joel 2/19/24: using ConcurrentSkipListMap here to avoid concurrent
    // modification exception; used to be TreeMap.
    private final NavigableMap<Double, T> m_pastSnapshots = new ConcurrentSkipListMap<>();

    public TimeInterpolatableBuffer100(double historyS, double timeS, T initialValue) {
        m_historyS = historyS;
        m_pastSnapshots.put(timeS, initialValue);
    }

    /**
     * Remove stale entries and add the new one.
     */
    public void put(double timeS, T value) {
        trim(timeS);
        m_pastSnapshots.put(timeS, value);
    }

    /**
     * Remove all entries and add the new one.
     */
    public void reset(double timeS, T value) {
        m_pastSnapshots.clear();
        m_pastSnapshots.put(timeS, value);
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

        Entry<Double, T> topBound = m_pastSnapshots.ceilingEntry(timeSeconds);
        Entry<Double, T> bottomBound = m_pastSnapshots.floorEntry(timeSeconds);

        // Return the opposite bound if the other is null
        if (topBound == null) {
            t.log(Level.TRACE, "buffer", "bottom", bottomBound.getValue().toString());
            return bottomBound.getValue();
        }
        if (bottomBound == null) {
            t.log(Level.TRACE, "buffer", "top", topBound.getValue().toString());
            return topBound.getValue();
        }

        // If both bounds exist, interpolate between them.
        // Because T is between [0, 1], we want the ratio of
        // (the difference between the current time and bottom bound) and (the
        // difference between top and bottom bounds).

        t.log(Level.TRACE, "buffer", "bottom", bottomBound.getValue().toString());
        t.log(Level.TRACE, "buffer", "top", topBound.getValue().toString());
        double timeSinceBottom = timeSeconds - bottomBound.getKey();
        double timeSpan = topBound.getKey() - bottomBound.getKey();
        double timeFraction = timeSinceBottom / timeSpan;
        return bottomBound.getValue().interpolate(topBound.getValue(), timeFraction);
    }

    public SortedMap<Double, T> tailMap(double t, boolean inclusive) {
        return m_pastSnapshots.tailMap(t, inclusive);
    }

    /** The most recent timestampSeconds. */
    public double lastKey() {
        // consider caching this at write time
        return m_pastSnapshots.lastKey();
    }

    /** The most recent entry. */
    public Entry<Double, T> lastEntry() {
        // consider caching this at write time
        return m_pastSnapshots.lastEntry();
    }

    public Entry<Double, T> lowerEntry(Double t) {
        return m_pastSnapshots.lowerEntry(t);
    }

    public Entry<Double, T> ceilingEntry(Double arg0) {
        return m_pastSnapshots.ceilingEntry(arg0);
    }

    //////////////////////////////

    /**
     * Removes samples older than the history limit.
     *
     * @param timeS The current timestamp.
     */
    private void trim(double timeS) {
        while (!m_pastSnapshots.isEmpty()) {
            Entry<Double, T> oldest = m_pastSnapshots.firstEntry();
            Double oldestTimeS = oldest.getKey();
            double oldestAgeS = timeS - oldestTimeS;
            // if oldest is younger than the history limit, we're done
            if (oldestAgeS < m_historyS)
                return;
            m_pastSnapshots.remove(oldestTimeS);
        }
    }

}
