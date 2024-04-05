package org.team100.lib.copies;

import java.util.NavigableMap;
import java.util.Optional;
import java.util.SortedMap;
import java.util.Map.Entry;
import java.util.concurrent.ConcurrentSkipListMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;

/**
 * Uses an Interpolator to provide interpolated sampling with a history limit.
 */
public final class TimeInterpolatableBuffer100<T> {
    private final double m_historySize;
    private final Interpolator<T> m_interpolatingFunc;
    // @joel 2/19/24: using ConcurrentSkipListMap here to avoid concurrent
    // modification exception; used to be TreeMap.
    private final NavigableMap<Double, T> m_pastSnapshots = new ConcurrentSkipListMap<>();

    private TimeInterpolatableBuffer100(
            Interpolator<T> interpolateFunction,
            double historySizeSeconds) {
        m_historySize = historySizeSeconds;
        m_interpolatingFunc = interpolateFunction;
    }

    public static <T> TimeInterpolatableBuffer100<T> createBuffer(
            Interpolator<T> interpolateFunction,
            double historySizeSeconds) {
        return new TimeInterpolatableBuffer100<>(
                interpolateFunction,
                historySizeSeconds);
    }

    public static <T extends Interpolatable<T>> TimeInterpolatableBuffer100<T> createBuffer(
            double historySizeSeconds) {
        return new TimeInterpolatableBuffer100<>(
                Interpolatable::interpolate,
                historySizeSeconds);
    }

    public static TimeInterpolatableBuffer100<Double> createDoubleBuffer(
            double historySizeSeconds) {
        return new TimeInterpolatableBuffer100<>(
                MathUtil::interpolate,
                historySizeSeconds);
    }

    public void addSample(double timeSeconds, T sample) {
        cleanUp(timeSeconds);
        m_pastSnapshots.put(timeSeconds, sample);
    }

    /**
     * Removes samples older than the history limit.
     *
     * @param time The current timestamp.
     */
    private void cleanUp(double time) {
        while (!m_pastSnapshots.isEmpty()) {
            var entry = m_pastSnapshots.firstEntry();
            if (time - entry.getKey() >= m_historySize) {
                m_pastSnapshots.remove(entry.getKey());
            } else {
                return;
            }
        }
    }

    /** Clears history entirely. */
    public void clear() {
        m_pastSnapshots.clear();
    }

    /**
     * Sample the buffer at the given time. If the buffer is empty, an empty
     * Optional is returned.
     */
    public Optional<T> getSample(double timeSeconds) {
        if (m_pastSnapshots.isEmpty()) {
            return Optional.empty();
        }

        // Special case for when the requested time is the same as a sample
        var nowEntry = m_pastSnapshots.get(timeSeconds);
        if (nowEntry != null) {
            return Optional.of(nowEntry);
        }

        var topBound = m_pastSnapshots.ceilingEntry(timeSeconds);
        var bottomBound = m_pastSnapshots.floorEntry(timeSeconds);

        // Return null if neither sample exists, and the opposite bound if the other is
        // null
        if (topBound == null && bottomBound == null) {
            return Optional.empty();
        } else if (topBound == null) {
            return Optional.of(bottomBound.getValue());
        } else if (bottomBound == null) {
            return Optional.of(topBound.getValue());
        } else {
            // Otherwise, interpolate. Because T is between [0, 1], we want the ratio of
            // (the difference between the current time and bottom bound) and (the
            // difference between top and bottom bounds).
            return Optional.of(
                    m_interpolatingFunc.interpolate(
                            bottomBound.getValue(),
                            topBound.getValue(),
                            (timeSeconds - bottomBound.getKey()) / (topBound.getKey() - bottomBound.getKey())));
        }
    }

    public SortedMap<Double, T> tailMap(double t, boolean inclusive) {
        return m_pastSnapshots.tailMap(t, inclusive);
    }

    /** The most recent timestampSeconds. */
    public double lastKey() {
        // consider caching this at write time
        return m_pastSnapshots.lastKey();
    }

    /** The more recent entry. */
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
    

}
