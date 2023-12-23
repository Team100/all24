package org.team100.lib.swerve;

/**
 * Enforces a fixed limit on delta v.
 * 
 * TODO: make the limit configurable (e.g. dependent on elevator extension)
 * TODO: make this depend on direction (e.g. CG not in center)
 */
public class CentripetalAccelerationLimiter {
    private static final double kDtSec = 0.02;

    private final SwerveKinematicLimits m_limits;

    public CentripetalAccelerationLimiter(SwerveKinematicLimits limits) {
        m_limits = limits;
    }

    public double enforceCentripetalLimit(
            double[] prev_vx,
            double[] prev_vy,
            double[] desired_vx,
            double[] desired_vy,
            double min_s) {
        for (int i = 0; i < prev_vx.length; ++i) {
            double prev_vx_i = prev_vx[i];
            double prev_vy_i = prev_vy[i];
            double desired_vx_i = desired_vx[i];
            double desired_vy_i = desired_vy[i];
            double dv = Math.hypot(desired_vx_i - prev_vx_i, desired_vy_i - prev_vy_i);
            double s = kDtSec * m_limits.kMaxCentripetalAccel / dv;
            min_s = Math.min(min_s, s);
        }
        return min_s;
    }
}
