package org.team100.lib.path;

/**
 * Allows sampling a path by index.
 */
public class PathIndexSampler {
    private final Path100 m_trajectory;

    public PathIndexSampler(Path100 trajectory) {
        m_trajectory = trajectory;
    }

    public PathSamplePoint sample(double index) {
        return m_trajectory.getInterpolated(index);
    }

    public double getMaxIndex() {
        if (m_trajectory.isEmpty())
            return 0.0;
        return m_trajectory.length() - 1.0;
    }

    public double getMinIndex() {
        return 0.0;
    }
}