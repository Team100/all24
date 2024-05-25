package org.team100.lib.path;

import org.team100.lib.timing.TimingUtil;

/**
 * Allows sampling a path by distance along it.
 * 
 * TODO: remove this class, use the index sampler instead.
 * this doesn't work correctly for holonomic paths (e.g. "rotate in place")
 */
public class PathDistanceSampler {
    private final Path100 m_path;
    /** in meters */
    private final double[] m_distances;

    public PathDistanceSampler(final Path100 path) {
        m_path = path;
        m_distances = new double[m_path.length()];
        m_distances[0] = 0.0;
        for (int i = 1; i < m_path.length(); ++i) {
            m_distances[i] = m_distances[i - 1]
                    + m_path.getPoint(i - 1).state().distance(m_path.getPoint(i).state());
        }
    }

    /**
     * @param distance in meters
     */
    public PathSamplePoint sample(double distance) throws TimingUtil.TimingException {
        if (distance >= getMaxDistance()) {
            PathPoint point = m_path.getPoint(m_path.length() - 1);
            return new PathSamplePoint(point.state(), point.index(), point.index());
        }
        if (distance <= 0.0) {
            PathPoint point = m_path.getPoint(0);
            return new PathSamplePoint(point.state(), point.index(), point.index());
        }
        for (int i = 1; i < m_distances.length; ++i) {
            final PathPoint point = m_path.getPoint(i);
            if (m_distances[i] >= distance) {
                final PathPoint prev_s = m_path.getPoint(i - 1);
                if (Math.abs(m_distances[i] - m_distances[i - 1]) <= 1e-12) {
                    return new PathSamplePoint(point.state(), point.index(), point.index());
                } else {
                    return new PathSamplePoint(
                            prev_s.state().interpolate(point.state(),
                                    (distance - m_distances[i - 1]) / (m_distances[i] - m_distances[i - 1])),
                            i - 1, i);
                }
            }
        }
        throw new TimingUtil.TimingException();
    }

    public double getMaxDistance() {
        return m_distances[m_distances.length - 1];
    }

    public double getMinDistance() {
        return 0.0;
    }
}
