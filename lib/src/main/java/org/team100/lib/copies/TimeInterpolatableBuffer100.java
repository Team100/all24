package org.team100.lib.copies;

import java.util.NavigableMap;
import java.util.Optional;
import java.util.SortedMap;
import java.util.concurrent.ConcurrentSkipListMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;

/**
 * The TimeInterpolatableBuffer provides an easy way to estimate past
 * measurements. One application
 * might be in conjunction with the DifferentialDrivePoseEstimator, where
 * knowledge of the robot
 * pose at the time when vision or other global measurement were recorded is
 * necessary, or for
 * recording the past angles of mechanisms as measured by encoders.
 *
 * @param <T> The type stored in this buffer.
 */
public final class TimeInterpolatableBuffer100<T> {
    private final double m_historySize;
    private final Interpolator<T> m_interpolatingFunc;
    // @joel 2/19/24: using ConcurrentSkipListMap here to avoid concurrent modification exception; used to be TreeMap.
    private final NavigableMap<Double, T> m_pastSnapshots = new ConcurrentSkipListMap<>();

    private TimeInterpolatableBuffer100(Interpolator<T> interpolateFunction, double historySizeSeconds) {
        this.m_historySize = historySizeSeconds;
        this.m_interpolatingFunc = interpolateFunction;
    }

    /**
     * Create a new TimeInterpolatableBuffer.
     *
     * @param interpolateFunction The function used to interpolate between values.
     * @param historySizeSeconds  The history size of the buffer.
     * @param <T>                 The type of data to store in the buffer.
     * @return The new TimeInterpolatableBuffer.
     */
    public static <T> TimeInterpolatableBuffer100<T> createBuffer(
            Interpolator<T> interpolateFunction, double historySizeSeconds) {
        return new TimeInterpolatableBuffer100<>(interpolateFunction, historySizeSeconds);
    }

    /**
     * Create a new TimeInterpolatableBuffer that stores a given subclass of
     * {@link Interpolatable}.
     *
     * @param historySizeSeconds The history size of the buffer.
     * @param <T>                The type of {@link Interpolatable} to store in the
     *                           buffer.
     * @return The new TimeInterpolatableBuffer.
     */
    public static <T extends Interpolatable<T>> TimeInterpolatableBuffer100<T> createBuffer(
            double historySizeSeconds) {
        return new TimeInterpolatableBuffer100<>(Interpolatable::interpolate, historySizeSeconds);
    }

    /**
     * Create a new TimeInterpolatableBuffer to store Double values.
     *
     * @param historySizeSeconds The history size of the buffer.
     * @return The new TimeInterpolatableBuffer.
     */
    public static TimeInterpolatableBuffer100<Double> createDoubleBuffer(double historySizeSeconds) {
        return new TimeInterpolatableBuffer100<>(MathUtil::interpolate, historySizeSeconds);
    }

    /**
     * Add a sample to the buffer.
     *
     * @param timeSeconds The timestamp of the sample.
     * @param sample      The sample object.
     */
    public void addSample(double timeSeconds, T sample) {
        cleanUp(timeSeconds);
        m_pastSnapshots.put(timeSeconds, sample);
    }

    /**
     * Removes samples older than our current history size.
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

    /** Clear all old samples. */
    public void clear() {
        m_pastSnapshots.clear();
    }

    /**
     * Sample the buffer at the given time. If the buffer is empty, an empty
     * Optional is returned.
     *
     * @param timeSeconds The time at which to sample.
     * @return The interpolated value at that timestamp or an empty Optional.
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
            // (the difference
            // between the current time and bottom bound) and (the difference between top
            // and bottom
            // bounds).
            return Optional.of(
                    m_interpolatingFunc.interpolate(
                            bottomBound.getValue(),
                            topBound.getValue(),
                            (timeSeconds - bottomBound.getKey()) / (topBound.getKey() - bottomBound.getKey())));
        }
    }

    public SortedMap<Double, T> tailMap(double t) {
        return m_pastSnapshots.tailMap(t);
    }

    public double lastKey() {
        return m_pastSnapshots.lastKey();
    }
}
