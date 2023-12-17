package org.team100.lib.trajectory;

import java.util.Optional;

import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.MathUtil;

/**
 * Allows iterating over the schedule of a trajectory.
 */
public class TrajectoryTimeIterator {
    private final TrajectoryTimeSampler m_sampler;
    /** progress along the trajectory in seconds */
    private double m_timeS = 0.0;
    private Optional<TrajectorySamplePoint> m_current;

    public TrajectoryTimeIterator(TrajectoryTimeSampler sampler) {
        m_sampler = sampler;
        // No effect if view is empty.
        m_current = m_sampler.sample(m_sampler.getStartS());
        m_timeS = m_sampler.getStartS();
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getProgress() {
        return m_timeS;
    }

    public double getRemainingProgress() {
        return Math.max(0.0, m_sampler.getEndS() - m_timeS);
    }

    public Optional<TrajectorySamplePoint> getSample() {
        return m_current;
    }

    public Optional<TimedPose> getState() {
        Optional<TrajectorySamplePoint> sample = getSample();
        if (sample.isPresent())
            return Optional.of(sample.get().state());
        return Optional.empty();
    }

    /**
     * Sample the trajectory and update the iterator state.
     * 
     * @param additional_progress in seconds
     * @return a sample point, or empty if something went wrong.
     */
    public Optional<TrajectorySamplePoint> advance(double additional_progress) {
        m_timeS = MathUtil.clamp(m_timeS + additional_progress, m_sampler.getStartS(), m_sampler.getEndS());
        m_current = m_sampler.sample(m_timeS);
        return m_current;
    }

    /**
     * Sample the trajectory without changing the iterator state.
     * 
     * @param additional_progress in seconds
     * @return a sample point, or empty if something went wrong.
     */
    public Optional<TrajectorySamplePoint> preview(double additional_progress) {
        if (Double.isNaN(additional_progress))
            throw new IllegalArgumentException("additional_progress is NaN");
        final double progress = Math.max(m_sampler.getStartS(),
                Math.min(m_sampler.getEndS(), m_timeS + additional_progress));
        return m_sampler.sample(progress);
    }

    public Trajectory100 trajectory() {
        return m_sampler.trajectory();
    }
}
