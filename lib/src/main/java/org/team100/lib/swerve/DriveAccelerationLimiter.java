package org.team100.lib.swerve;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleSupplierLogger2;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Math100;

/**
 * Enforces drive motor torque constraints.
 */
public class DriveAccelerationLimiter implements Glassy {
    private static final int kMaxIterations = 10;

    private final SwerveKinodynamics m_limits;
    // LOGGERS
    private final DoubleSupplierLogger2 m_log_max_step;
    private final DoubleSupplierLogger2 m_log_s;

    public DriveAccelerationLimiter(LoggerFactory parent, SwerveKinodynamics limits) {
        m_limits = limits;
        LoggerFactory child = parent.child(this);
        m_log_max_step = child.doubleLogger(Level.TRACE, "max_vel_step");
        m_log_s = child.doubleLogger(Level.TRACE, "s");
    }

    public double enforceWheelAccelLimit(
            double[] prev_vx,
            double[] prev_vy,
            double[] desired_vx,
            double[] desired_vy) {
        double min_s = 1.0;
        for (int i = 0; i < prev_vx.length; ++i) {
            double max_vel_step = SwerveUtil.getMaxVelStep(
                    m_limits,
                    prev_vx[i],
                    prev_vy[i],
                    desired_vx[i],
                    desired_vy[i]);
            m_log_max_step.log(() -> max_vel_step);

            // reduces the size of the search space if min_s is already constrained (by
            // earlier modules)
            double vx_min_s = Math100.interpolate(prev_vx[i], desired_vx[i], min_s);
            double vy_min_s = Math100.interpolate(prev_vy[i], desired_vy[i], min_s);

            double s = SwerveUtil.findDriveMaxS(
                    prev_vx[i],
                    prev_vy[i],
                    vx_min_s,
                    vy_min_s,
                    max_vel_step,
                    kMaxIterations);
            min_s = Math.min(min_s, s);
            if (min_s == 0.0) {
                break;
            }
        }
        final double s = min_s;
        m_log_s.log(() -> s);
        return min_s;
    }

}
