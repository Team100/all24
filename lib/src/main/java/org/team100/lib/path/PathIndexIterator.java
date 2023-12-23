package org.team100.lib.path;

import org.team100.lib.geometry.Pose2dWithMotion;

/**
 * Allows iterating over the index points of a path.
 */
public class PathIndexIterator {
    private final PathIndexSampler m_sampler;
    private double m_progress = 0.0;
    private PathSamplePoint m_sample;

    public PathIndexIterator(final PathIndexSampler view) {
        m_sampler = view;

        // No effect if view is empty.
        m_sample = m_sampler.sample(m_sampler.getMinIndex());
        m_progress = m_sampler.getMinIndex();
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getProgress() {
        return m_progress;
    }

    public double getRemainingProgress() {
        return Math.max(0.0, m_sampler.getMaxIndex() - m_progress);
    }

    public PathSamplePoint getSample() {
        return m_sample;
    }

    public Pose2dWithMotion getState() {
        return getSample().state();
    }

    public PathSamplePoint advance(double additional_progress) {
        m_progress = Math.max(m_sampler.getMinIndex(),
                Math.min(m_sampler.getMaxIndex(), m_progress + additional_progress));
        m_sample = m_sampler.sample(m_progress);
        return m_sample;
    }

    public PathSamplePoint preview(double additional_progress) {
        final double progress = Math.max(m_sampler.getMinIndex(),
                Math.min(m_sampler.getMaxIndex(), m_progress + additional_progress));
        return m_sampler.sample(progress);
    }
}
