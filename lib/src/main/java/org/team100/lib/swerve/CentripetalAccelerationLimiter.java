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

    public double enforceCentripetalLimit(double dx, double dy, double min_s) {
        double dv = Math.hypot(dx, dy);
        double s = kDtSec * m_limits.kMaxCentripetalAccel / dv;
        return Math.min(min_s, s);
    }
}
