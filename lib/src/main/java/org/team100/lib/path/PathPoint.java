package org.team100.lib.path;

import org.team100.lib.geometry.Pose2dWithMotion;

/**
 * Represents a point on a 2d path with heading.
 * 
 * There's no timing data here; for that, see TrajectoryPoint.
 */
public class PathPoint {
    private final Pose2dWithMotion m_state;
    private final int m_index;

    public PathPoint(final Pose2dWithMotion state, int index) {
        m_state = state;
        m_index = index;
    }

    public Pose2dWithMotion state() {
        return m_state;
    }

    public int index() {
        return m_index;
    }
}
