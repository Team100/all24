package org.team100.lib.timing;

import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A constant velocity limit within a rectangle; no limit outside.
 */
public class VelocityLimitRegionConstraint implements TimingConstraint {
    private final Translation2d m_min;
    private final Translation2d m_max;
    private final double m_limit;

    public VelocityLimitRegionConstraint(
            Translation2d min_corner,
            Translation2d max_corner,
            double velocity_limit) {
        if (velocity_limit < 0)
            throw new IllegalArgumentException();
        m_min = min_corner;
        m_max = max_corner;
        m_limit = velocity_limit;
    }

    @Override
    public NonNegativeDouble getMaxVelocity(Pose2dWithMotion state) {
        final Translation2d translation = state.getTranslation();
        if (translation.getX() <= m_max.getX() && translation.getX() >= m_min.getX() &&
                translation.getY() <= m_max.getY() && translation.getY() >= m_min.getY()) {
            return new NonNegativeDouble(m_limit);
        }
        return new NonNegativeDouble(Double.POSITIVE_INFINITY);
    }

    @Override
    public TimingConstraint.MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state,
            double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }

}