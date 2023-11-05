package org.team100.lib.trajectory;

/**
 * Allows sampling a trajectory by its schedule.
 * Derived from 254 TimedView.
 */
public class TrajectoryTimeSampler {
    public static class TrajectoryTimeSampleException extends RuntimeException {}

    protected final Trajectory trajectory_;
    protected final double startTimeS;
    protected final double endTimeS;

    public TrajectoryTimeSampler(Trajectory trajectory) {
        trajectory_ = trajectory;
        startTimeS = trajectory_.getPoint(0).state().getTimeS();
        endTimeS = trajectory_.getPoint(trajectory_.length() - 1).state().getTimeS();
    }

    public double getStartTimeS() {
        return startTimeS;
    }

    public double getEndTimeS() {
        return endTimeS;
    }

    /**
     * @param timeS seconds
     */
    public TrajectorySamplePoint sample(double timeS) {
        if (timeS >= endTimeS) {
            TrajectoryPoint point = trajectory_.getPoint(trajectory_.length() - 1);
            return new TrajectorySamplePoint(point.state(), point.index(), point.index());
        }
        if (timeS <= startTimeS) {
            TrajectoryPoint point = trajectory_.getPoint(0);
            return new TrajectorySamplePoint(point.state(), point.index(), point.index());
        }
        for (int i = 1; i < trajectory_.length(); ++i) {
            final TrajectoryPoint point = trajectory_.getPoint(i);
            if (point.state().getTimeS() >= timeS) {
                final TrajectoryPoint prev_s = trajectory_.getPoint(i - 1);
                if (Math.abs(point.state().getTimeS() - prev_s.state().getTimeS()) <= 1e-12) {
                    return new TrajectorySamplePoint(point.state(), point.index(), point.index());
                }
                return new TrajectorySamplePoint(
                        prev_s.state().interpolate2(point.state(),
                                (timeS - prev_s.state().getTimeS())
                                        / (point.state().getTimeS() - prev_s.state().getTimeS())),
                        i - 1, i);
            }
        }
        throw new TrajectoryTimeSampleException();
    }

    public Trajectory trajectory() {
        return trajectory_;
    }
}
