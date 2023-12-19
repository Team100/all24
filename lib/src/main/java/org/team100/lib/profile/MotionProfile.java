package org.team100.lib.profile;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

/**
 * A one-dimensional motion profile.
 * 
 * Consists of a list of motion segments.
 */
public class MotionProfile {
    private final List<MotionSegment> m_segments;
    private final double m_duration;

    /**
     * Trapezoidal motion profile composed of motion segments.
     *
     * @param segments profile motion segments
     */
    public MotionProfile(List<MotionSegment> segments) {
        if (segments.isEmpty())
            throw new IllegalArgumentException();
        m_segments = segments;
        m_duration = m_segments.stream().map((it) -> it.getDt()).reduce(0.0, Double::sum);
    }

    /**
     * Returns the [MotionState] at time [t].
     */
    public MotionState get(double t) {
        if (t < 0.0)
            return m_segments.get(0).getStart().stationary();

        var remainingTime = t;
        for (MotionSegment segment : m_segments) {
            if (remainingTime <= segment.getDt()) {
                return segment.get(remainingTime);
            }
            remainingTime -= segment.getDt();
        }

        return m_segments.get(m_segments.size() - 1).end().stationary();
    }

    /**
     * Returns the duration of the motion profile.
     */
    public double duration() {
        return m_duration;
    }

    /**
     * Returns a reversed version of the motion profile.
     */
    public MotionProfile reversed() {
        List<MotionSegment> l = m_segments.stream().map((it) -> it.reversed()).collect(Collectors.toList());
        Collections.reverse(l);
        return new MotionProfile(l);
    }

    /**
     * Returns a flipped version of the motion profile.
     */
    public MotionProfile flipped() {
        return new MotionProfile(m_segments.stream().map((it) -> it.flipped()).collect(Collectors.toList()));
    }

    /**
     * Returns the start [MotionState].
     */
    public MotionState start() {
        return m_segments.get(0).getStart();
    }

    /**
     * Returns the end [MotionState].
     */
    public MotionState end() {
        return m_segments.get(m_segments.size() - 1).end();
    }

    /**
     * Returns a new motion profile with [other] concatenated.
     */
    MotionProfile plus(MotionProfile other) {
        MotionProfileBuilder builder = new MotionProfileBuilder(start());
        builder.appendProfile(this);
        builder.appendProfile(other);
        return builder.build();
    }

    // for testing and building only
    List<MotionSegment> getSegments() {
        return m_segments;
    }

}
