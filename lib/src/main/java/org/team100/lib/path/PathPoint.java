package org.team100.lib.path;

import org.team100.lib.geometry.Pose2dWithMotion;

/**
 * Represents a point on a 2d path with heading.
 * 
 * There's no timing data here; for that, see TrajectoryPoint.
 */
public class PathPoint {
    private final Pose2dWithMotion state_;
    private final int index_;

    public PathPoint(final Pose2dWithMotion state, int index) {
        state_ = state;
        index_ = index;
    }

    public Pose2dWithMotion state() {
        return state_;
    }

    public int index() {
        return index_;
    }
}
