package org.team100.lib.path;

/**
 * Allows sampling a path by index.
 */
public class PathIndexSampler {
    private final Path100 m_path;

    public PathIndexSampler(Path100 path) {
        m_path = path;
    }

    public PathSamplePoint sample(double index) {
        return m_path.getInterpolated(index);
    }

    public double getMaxIndex() {
        if (m_path.isEmpty())
            return 0.0;
        return m_path.length() - 1.0;
    }

    public double getMinIndex() {
        return 0.0;
    }
}