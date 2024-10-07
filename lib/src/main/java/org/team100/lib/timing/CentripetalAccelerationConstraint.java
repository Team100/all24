package org.team100.lib.timing;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

/**
 * Velocity limit based on curvature and centripetal acceleration limit, which
 * is taken to be the capsize limit (scaled).
 */
public class CentripetalAccelerationConstraint implements TimingConstraint {
    // m/s^2
    final double mMaxCentripetalAccel;

    /**
     * Use the factory.
     * 
     * @param limits absolute maxima
     * @param scale  apply to the maximum capsize accel to get the actual
     *               constraint. this is useful to slow down trajectories in
     *               sharp curves, which makes odometry more accurate and reduces
     *               the effect of steering lag.
     */
    public CentripetalAccelerationConstraint(SwerveKinodynamics limits, double scale) {
        mMaxCentripetalAccel = limits.getMaxCapsizeAccelM_S2() * scale;
    }

    @Override
    public NonNegativeDouble getMaxVelocity(final Pose2dWithMotion state) {
        return new NonNegativeDouble(Math.sqrt(Math.abs(mMaxCentripetalAccel / state.getCurvature())));
    }

    /**
     * The capsize limit is not applied to acceleration along the path. This
     * constraint only handles "cross track" acceleration.
     */
    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }
}
