package org.team100.lib.timing;

import org.team100.lib.geometry.Pose2dWithMotion;

public interface TimingConstraint {
    double getMaxVelocity(Pose2dWithMotion state);

    MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocity);

    class MinMaxAcceleration {
        public static final MinMaxAcceleration kNoLimits = new MinMaxAcceleration();

        private final double min_acceleration_;
        private final double max_acceleration_;

        private MinMaxAcceleration() {
            this(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        public MinMaxAcceleration(double min_acceleration, double max_acceleration) {
            if (min_acceleration() > max_acceleration()) {
                throw new IllegalArgumentException();
            }
            min_acceleration_ = min_acceleration;
            max_acceleration_ = max_acceleration;
        }

        public double min_acceleration() {
            return min_acceleration_;
        }

        public double max_acceleration() {
            return max_acceleration_;
        }
    }
}
