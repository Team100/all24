package org.team100.lib.swerve;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

/**
 * Enforces a fixed limit on delta v.
 */
public class CapsizeAccelerationLimiter {

    private final SwerveKinodynamics m_limits;

    public CapsizeAccelerationLimiter(SwerveKinodynamics limits) {
        m_limits = limits;
    }

    public double enforceCentripetalLimit(double dx, double dy, double min_s, double kDtSec) {
        double dv = Math.hypot(dx, dy);
        double s = kDtSec * m_limits.getMaxCapsizeAccelM_S2() / dv;
        return Math.min(min_s, s);
    }
}
