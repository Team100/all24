package org.team100.lib.path;

import org.team100.lib.geometry.Pose2dWithMotion;

/**
 * Represents a sample of a 2d path with heading.
 * 
 * Includes the interpolation index bounds, for testing.
 */
public class PathSamplePoint {
    private final Pose2dWithMotion m_state;
    private final int m_indexFloor;
    private final int m_IndexCeil;

    public PathSamplePoint(Pose2dWithMotion state, int index_floor, int index_ceil) {
        m_state = state;
        m_indexFloor = index_floor;
        m_IndexCeil = index_ceil;
    }

    public Pose2dWithMotion state() {
        return m_state;
    }

    /** for testing */
    int getIndexFloor() {
        return m_indexFloor;
    }

    /** for testing */
    int getIndexCeil() {
        return m_IndexCeil;
    }
}
