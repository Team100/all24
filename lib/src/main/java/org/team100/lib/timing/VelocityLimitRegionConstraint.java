package org.team100.lib.timing;

import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Translation2d;

public class VelocityLimitRegionConstraint implements TimingConstraint {
    protected final Translation2d min_corner_;
    protected final Translation2d max_corner_;
    protected final double velocity_limit_;

    public VelocityLimitRegionConstraint(Translation2d min_corner, Translation2d max_corner, double velocity_limit) {
        min_corner_ = min_corner;
        max_corner_ = max_corner;
        velocity_limit_ = velocity_limit;
    }

    @Override
    public double getMaxVelocity(Pose2dWithMotion state) {
        final Translation2d translation = state.getTranslation();
        if (translation.getX() <= max_corner_.getX() && translation.getX() >= min_corner_.getX() &&
                translation.getY() <= max_corner_.getY() && translation.getY() >= min_corner_.getY()) {
            return velocity_limit_;
        }
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public TimingConstraint.MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state,
            double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }

}