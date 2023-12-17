package org.team100.lib.trajectory;

import java.util.Optional;

/**
 * Allows sampling a trajectory by its schedule.
 * Derived from 254 TimedView.
 */
public class TrajectoryTimeSampler {
    private final Trajectory100 m_trajectory;
    private final double m_startS;
    private final double m_endS;

    public TrajectoryTimeSampler(Trajectory100 trajectory) {
        m_trajectory = trajectory;
        m_startS = m_trajectory.getPoint(0).state().getTimeS();
        m_endS = m_trajectory.getPoint(m_trajectory.length() - 1).state().getTimeS();
    }

    public double getStartS() {
        return m_startS;
    }

    public double getEndS() {
        return m_endS;
    }

    /**
     * Returns empty if no sample can be found.  This shouldn't happen, but if it
     * does, there's no reasonable default.
     * 
     * @param timeS seconds
     */
    public Optional<TrajectorySamplePoint> sample(double timeS) {
        if (Double.isNaN(timeS)) {
            throw new IllegalArgumentException("time is NaN");
        }
        if (timeS >= m_endS) {
            TrajectoryPoint point = m_trajectory.getPoint(m_trajectory.length() - 1);
            return Optional.of(new TrajectorySamplePoint(point.state(), point.index(), point.index()));
        }
        if (timeS <= m_startS) {
            TrajectoryPoint point = m_trajectory.getPoint(0);
            return Optional.of(new TrajectorySamplePoint(point.state(), point.index(), point.index()));
        }
        for (int i = 1; i < m_trajectory.length(); ++i) {
            final TrajectoryPoint point = m_trajectory.getPoint(i);
            if (point.state().getTimeS() >= timeS) {
                final TrajectoryPoint prev_s = m_trajectory.getPoint(i - 1);
                if (Math.abs(point.state().getTimeS() - prev_s.state().getTimeS()) <= 1e-12) {
                    return Optional.of(new TrajectorySamplePoint(point.state(), point.index(), point.index()));
                }
                return Optional.of(new TrajectorySamplePoint(
                        prev_s.state().interpolate2(point.state(),
                                (timeS - prev_s.state().getTimeS())
                                        / (point.state().getTimeS() - prev_s.state().getTimeS())),
                        i - 1, i));
            }
        }
        return Optional.empty();
    }

    public Trajectory100 trajectory() {
        return m_trajectory;
    }

    @Override
    public String toString() {
        return "TrajectoryTimeSampler [trajectory_=" + m_trajectory + ", startTimeS=" + m_startS + ", endTimeS="
                + m_endS + "]";
    }
}
