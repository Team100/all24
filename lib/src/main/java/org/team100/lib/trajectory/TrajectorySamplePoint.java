package org.team100.lib.trajectory;

import org.team100.lib.timing.TimedPose;

/** Represents a sample of a 2d path including heading and a schedule. */
public class TrajectorySamplePoint {
    private final TimedPose m_state;
    private final int m_indexFloor;
    private final int m_indexCeil;

    public TrajectorySamplePoint(final TimedPose state, int index_floor, int index_ceil) {
        m_state = state;
        m_indexFloor = index_floor;
        m_indexCeil = index_ceil;
    }

    public TimedPose state() {
        return m_state;
    }

    /** For testing */
    int getIndexFloor() {
        return m_indexFloor;
    }

    /** For testing */
    int getIndexCeil() {
        return m_indexCeil;
    }

    @Override
    public String toString() {
        return "TrajectorySamplePoint [state_=" + m_state
                + ", index_floor_=" + m_indexFloor
                + ", index_ceil_=" + m_indexCeil + "]";
    }
}
