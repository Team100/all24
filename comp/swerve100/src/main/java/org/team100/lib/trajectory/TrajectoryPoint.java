package org.team100.lib.trajectory;

import org.team100.lib.timing.TimedPose;

/**
 * Represents a point and time on a 2d path with heading.
 */
public class TrajectoryPoint {
    private final TimedPose state_;
    private final int index_;

    public TrajectoryPoint(final TimedPose state, int index) {
        state_ = state;
        index_ = index;
    }

    public TimedPose state() {
        return state_;
    }

    public int index() {
        return index_;
    }
}
