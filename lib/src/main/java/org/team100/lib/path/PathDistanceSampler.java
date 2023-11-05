package org.team100.lib.path;

/**
 * Allows sampling a path by distance along it.
 */
public class PathDistanceSampler {
    private final Path trajectory_;
    private final double[] distances_;

    public PathDistanceSampler(final Path trajectory) {
        trajectory_ = trajectory;
        distances_ = new double[trajectory_.length()];
        distances_[0] = 0.0;
        for (int i = 1; i < trajectory_.length(); ++i) {
            distances_[i] = distances_[i - 1]
                    + trajectory_.getPoint(i - 1).state().distance(trajectory_.getPoint(i).state());
        }
    }

    /**
     * @param distance TODO what is the unit here?
     */
    public PathSamplePoint sample(double distance) {
        if (distance >= last_interpolant()) {
            PathPoint point = trajectory_.getPoint(trajectory_.length() - 1);
            return new PathSamplePoint(point.state(), point.index(), point.index());
        }
        if (distance <= 0.0) {
            PathPoint point = trajectory_.getPoint(0);
            return new PathSamplePoint(point.state(), point.index(), point.index());
        }
        for (int i = 1; i < distances_.length; ++i) {
            final PathPoint point = trajectory_.getPoint(i);
            if (distances_[i] >= distance) {
                final PathPoint prev_s = trajectory_.getPoint(i - 1);
                if (Math.abs(distances_[i] - distances_[i - 1]) <= 1e-12) {
                    return new PathSamplePoint(point.state(), point.index(), point.index());
                } else {
                    return new PathSamplePoint(
                            prev_s.state().interpolate(point.state(),
                                    (distance - distances_[i - 1]) / (distances_[i] - distances_[i - 1])),
                            i - 1, i);
                }
            }
        }
        throw new RuntimeException();
    }

    public double last_interpolant() {
        return distances_[distances_.length - 1];
    }

    public double first_interpolant() {
        return 0.0;
    }
}
