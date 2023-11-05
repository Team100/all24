package org.team100.lib.profile;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class MotionProfile {
    private final List<MotionSegment> segments;

    /**
     * Trapezoidal motion profile composed of motion segments.
     *
     * @param segments profile motion segments
     */
    public MotionProfile(List<MotionSegment> segments) {
        if (segments.isEmpty())
            throw new IllegalArgumentException();
        this.segments = segments;
    }

    /**
     * Returns the [MotionState] at time [t].
     */
    public MotionState get(double t) {
        if (t < 0.0)
            return segments.get(0).getStart().stationary();

        var remainingTime = t;
        for (MotionSegment segment : segments) {
            if (remainingTime <= segment.getDt()) {
                return segment.get(remainingTime);
            }
            remainingTime -= segment.getDt();
        }

        return segments.get(segments.size() - 1).end().stationary();
    }

    /**
     * Returns the duration of the motion profile.
     */
    public double duration() {
        return segments.stream().map((it) -> it.getDt()).reduce(0.0, Double::sum);
    }

    /**
     * Returns a reversed version of the motion profile.
     */
    public MotionProfile reversed() {
        List<MotionSegment> l = segments.stream().map((it) -> it.reversed()).collect(Collectors.toList());
        Collections.reverse(l);
        return new MotionProfile(l);
    }

    /**
     * Returns a flipped version of the motion profile.
     */
    public MotionProfile flipped() {
        return new MotionProfile(segments.stream().map((it) -> it.flipped()).collect(Collectors.toList()));
    }

    /**
     * Returns the start [MotionState].
     */
    public MotionState start() {
        return segments.get(0).getStart();
    }

    /**
     * Returns the end [MotionState].
     */
    public MotionState end() {
        return segments.get(segments.size() - 1).end();
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

    public List<MotionSegment> getSegments() {
        return segments;
    }

}
