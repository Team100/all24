package org.team100.lib.timing;

import org.team100.lib.geometry.Pose2dWithMotion;

/**
 *  Velocity limit based on curvature and centripetal acceleration limit.
 */
public class CentripetalAccelerationConstraint implements TimingConstraint {
    // m/s^2
    final double mMaxCentripetalAccel;

    public CentripetalAccelerationConstraint(final double max_centripetal_accel) {
        mMaxCentripetalAccel = max_centripetal_accel;
    }

    @Override
    public double getMaxVelocity(final Pose2dWithMotion state) {
        return Math.sqrt(Math.abs(mMaxCentripetalAccel / state.getCurvature()));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }
}
