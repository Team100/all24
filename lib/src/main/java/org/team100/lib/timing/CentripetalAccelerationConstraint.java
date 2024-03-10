package org.team100.lib.timing;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

/**
 * Velocity limit based on curvature and centripetal acceleration limit, which
 * is taken to be the capsize limit.
 */
public class CentripetalAccelerationConstraint implements TimingConstraint {
    // m/s^2
    final double mMaxCentripetalAccel;

    public CentripetalAccelerationConstraint(SwerveKinodynamics limits) {
        mMaxCentripetalAccel = limits.getMaxCapsizeAccelM_S2() * 0.2;
    }

    @Override
    public double getMaxVelocity(final Pose2dWithMotion state) {
        return Math.sqrt(Math.abs(mMaxCentripetalAccel / state.getCurvature()));
    }

    /**
     * The capsize limit is not applied here, this just handles "cross track"
     * acceleration.
     */
    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }
}
