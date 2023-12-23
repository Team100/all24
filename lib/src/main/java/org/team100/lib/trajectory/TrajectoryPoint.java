package org.team100.lib.trajectory;

import org.team100.lib.timing.TimedPose;

/**
 * Represents a point and time on a 2d path with heading.
 */
public class TrajectoryPoint {
    private final TimedPose m_state;
    private final int m_index;

    public TrajectoryPoint(final TimedPose state, int index) {
        m_state = state;
        m_index = index;
    }

    public TimedPose state() {
        return m_state;
    }

    public int index() {
        return m_index;
    }

    @Override
    public String toString() {
        return "TrajectoryPoint [state_=" + m_state + ", index_=" + m_index + "]";
    }
}
