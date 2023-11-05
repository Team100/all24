package org.team100.lib.path;

import org.team100.lib.geometry.Pose2dWithMotion;

/**
 * Allows iterating over the index points of a path.
 */
public class PathIndexIterator {
    protected final PathIndexSampler view_;
    protected double progress_ = 0.0;
    protected PathSamplePoint current_sample_;

    public PathIndexIterator(final PathIndexSampler view) {
        view_ = view;

        // No effect if view is empty.
        current_sample_ = view_.sample(view_.first_interpolant());
        progress_ = view_.first_interpolant();
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getProgress() {
        return progress_;
    }

    public double getRemainingProgress() {
        return Math.max(0.0, view_.last_interpolant() - progress_);
    }

    public PathSamplePoint getSample() {
        return current_sample_;
    }

    public Pose2dWithMotion getState() {
        return getSample().state();
    }

    public PathSamplePoint advance(double additional_progress) {
        progress_ = Math.max(view_.first_interpolant(),
                Math.min(view_.last_interpolant(), progress_ + additional_progress));
        current_sample_ = view_.sample(progress_);
        return current_sample_;
    }

    public PathSamplePoint preview(double additional_progress) {
        final double progress = Math.max(view_.first_interpolant(),
                Math.min(view_.last_interpolant(), progress_ + additional_progress));
        return view_.sample(progress);
    }
}
