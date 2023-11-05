package org.team100.lib.profile;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.util.MathUtil;

public class MotionProfileBuilder {
    private final List<MotionSegment> segments;
    private MotionState currentState;

    /**
     * Easy-to-use builder for creating motion profiles.
     *
     * @param start start motion state
     */
    public MotionProfileBuilder(MotionState start) {
        currentState = start;
        segments = new ArrayList<>();
    }

    /**
     * Appends a constant-jerk control for the provided duration.
     */
    MotionProfileBuilder appendJerkControl(double jerk, double dt) {
        MotionSegment segment = new MotionSegment(
                new MotionState(currentState.getX(), currentState.getV(), currentState.getA(), jerk), dt);
        segments.add(segment);
        currentState = segment.end();
        return this;
    }

    /**
     * Appends a constant-acceleration control for the provided duration.
     */
    MotionProfileBuilder appendAccelerationControl(double accel, double dt) {
        MotionSegment segment = new MotionSegment(new MotionState(currentState.getX(), currentState.getV(), accel), dt);
        segments.add(segment);
        currentState = segment.end();
        return this;
    }

    /**
     * Appends a [MotionProfile] to the current queue of control actions.
     */
    MotionProfileBuilder appendProfile(MotionProfile profile) {
        for (MotionSegment segment : profile.getSegments()) {
            if (MathUtil.epsilonEquals(segment.getStart().getJ(), 0.0)) {
                // constant acceleration
                appendAccelerationControl(segment.getStart().getA(), segment.getDt());
            } else {
                // constant jerk
                appendJerkControl(segment.getStart().getJ(), segment.getDt());
            }
        }
        return this;
    }

    /**
     * Constructs and returns the [MotionProfile] instance.
     */
    MotionProfile build() {
        return new MotionProfile(segments);
    }
}
