package org.team100.lib.trajectory;

import org.team100.lib.timing.TimedPose;

/** Represents a sample of a 2d path including heading and a schedule. */
public class TrajectorySamplePoint {
    protected final TimedPose state_;
    protected final int index_floor_;
    protected final int index_ceil_;

    public TrajectorySamplePoint(final TimedPose state, int index_floor, int index_ceil) {
        state_ = state;
        index_floor_ = index_floor;
        index_ceil_ = index_ceil;
    }

    public TimedPose state() {
        return state_;
    }

    public int index_floor() {
        return index_floor_;
    }

    public int index_ceil() {
        return index_ceil_;
    }

    @Override
    public String toString() {
        return "TrajectorySamplePoint [state_=" + state_
                + ", index_floor_=" + index_floor_
                + ", index_ceil_=" + index_ceil_ + "]";
    }
}
