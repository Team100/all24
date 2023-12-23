package org.team100.lib.timing;

import java.util.Optional;

import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Linear velocity limit based on spatial yaw rate and omega limit.
 */
public class YawRateConstraint implements TimingConstraint {
    private final double m_maxOmegaRad_S;

    public YawRateConstraint(final double maxOmegaRad_S) {
        m_maxOmegaRad_S = maxOmegaRad_S;
    }

    @Override
    public double getMaxVelocity(Pose2dWithMotion state) {
        Optional<Rotation2d> course = state.getCourse();
        if (course.isEmpty()) {
            // This is turn in place.
            return Double.MAX_VALUE;
        } else {
            // Heading rate in rad/m
            final double heading_rate = state.getHeadingRate();
            // rad/s / rad/m => m/s.
            return m_maxOmegaRad_S / Math.abs(heading_rate);
        }
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }
}