package org.team100.lib.trajectory;

import org.team100.lib.timing.TimedPose;

/** 
 * Allows iterating over the schedule of a trajectory. 
 * */
public class TrajectoryTimeIterator {
    protected final TrajectoryTimeSampler view_;
    /** progress along the trajectory in seconds */
    protected double progress_ = 0.0;
    protected TrajectorySamplePoint current_sample_;

    public TrajectoryTimeIterator() {
        view_=null;
    }

    public TrajectoryTimeIterator(final TrajectoryTimeSampler view) {
        view_ = view;
        System.out.printf("Time iterator with view: %s\n", view_);

        // No effect if view is empty.
        current_sample_ = view_.sample(view_.getStartTimeS());
        progress_ = view_.getStartTimeS();
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getProgress() {
        return progress_;
    }

    public double getRemainingProgress() {
        return Math.max(0.0, view_.getEndTimeS() - progress_);
    }

    public TrajectorySamplePoint getSample() {
        return current_sample_;
    }

    public TimedPose getState() {
        return getSample().state();
    }

    /** Sample the trajectory and update the iterator state.
     * @param additional_progress in seconds
     */
    public TrajectorySamplePoint advance(double additional_progress) {
        System.out.printf("sample additional progress (sec) %5.3f\n", additional_progress);
        progress_ = Math.max(view_.getStartTimeS(),
                Math.min(view_.getEndTimeS(), progress_ + additional_progress));
        current_sample_ = view_.sample(progress_);
        return current_sample_;
    }

    /** Sample the trajectory without changing the iterator state.
     * 
     * @param additional_progress in seconds
     */
    public TrajectorySamplePoint preview(double additional_progress) {
        System.out.printf("preview additional progress (sec) %5.3f\n", additional_progress);
        final double progress = Math.max(view_.getStartTimeS(),
                Math.min(view_.getEndTimeS(), progress_ + additional_progress));
        return view_.sample(progress);
    }

    public Trajectory100 trajectory() {
        return view_.trajectory();
    }
}
