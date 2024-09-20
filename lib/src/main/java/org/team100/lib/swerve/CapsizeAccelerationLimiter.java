package org.team100.lib.swerve;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry.Level;

/**
 * Enforces a fixed limit on delta v.
 */
public class CapsizeAccelerationLimiter implements Glassy {
    private final SupplierLogger2 m_logger;
    private final SwerveKinodynamics m_limits;

    public CapsizeAccelerationLimiter(SupplierLogger2 parent, SwerveKinodynamics limits) {
        m_logger = parent.child(this);
        m_limits = limits;
    }

    public double enforceCentripetalLimit(double dx, double dy, double kDtSec) {
        double min_s = 1.0;
        double dv = Math.hypot(dx, dy);
        if (Math.abs(dv) > 1e-6) {
            min_s = kDtSec * m_limits.getMaxCapsizeAccelM_S2() / dv;
        }
        double s = min_s;
        m_logger.doubleLogger(Level.TRACE, "s").log( () -> s);
        return s;
    }

    @Override
    public String getGlassName() {
        return "CapsizeAccelerationLimiter";
    }

}
