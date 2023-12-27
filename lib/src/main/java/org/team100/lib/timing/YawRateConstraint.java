package org.team100.lib.timing;

import java.util.Optional;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Linear velocity limit based on spatial yaw rate and drivetrain omega limit.
 * 
 * Slows the path velocity to accommodate the desired yaw rate.
 */
public class YawRateConstraint implements TimingConstraint {
    private final double m_maxOmegaRad_S;

    public YawRateConstraint(SwerveKinodynamics limits) {
        m_maxOmegaRad_S = limits.getMaxAngleSpeedRad_S();
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