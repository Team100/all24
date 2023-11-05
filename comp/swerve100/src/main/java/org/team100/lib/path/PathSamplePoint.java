package org.team100.lib.path;

import org.team100.lib.geometry.Pose2dWithMotion;

/**
 * Represents a sample of a 2d path with heading.
 */
public class PathSamplePoint {
    private final Pose2dWithMotion state_;
    private final int index_floor_;
    private final int index_ceil_;

    public PathSamplePoint(Pose2dWithMotion state, int index_floor, int index_ceil) {
        state_ = state;
        index_floor_ = index_floor;
        index_ceil_ = index_ceil;
    }

    public Pose2dWithMotion state() {
        return state_;
    }

    public int index_floor() {
        return index_floor_;
    }

    public int index_ceil() {
        return index_ceil_;
    }
}
