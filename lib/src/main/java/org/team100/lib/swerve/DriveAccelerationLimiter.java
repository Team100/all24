package org.team100.lib.swerve;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Names;

/**
 * Enforces drive motor torque constraints.
 */
public class DriveAccelerationLimiter implements Glassy {
    private static final Telemetry t = Telemetry.get();

    private static final int kMaxIterations = 10;

    private final SwerveKinodynamics m_limits;
    private final String m_name;

    public DriveAccelerationLimiter(String parent, SwerveKinodynamics limits) {
        m_limits = limits;
        m_name = Names.append(parent, this);
    }

    public double enforceWheelAccelLimit(
            double[] prev_vx,
            double[] prev_vy,
            double[] desired_vx,
            double[] desired_vy,
            double kDtSec) {
        double min_s = 1.0;
        // Enforce drive wheel acceleration limits.
        for (int i = 0; i < prev_vx.length; ++i) {
            double max_vel_step = SwerveUtil.getMaxVelStep(
                    m_limits,
                    prev_vx[i],
                    prev_vy[i],
                    desired_vx[i],
                    desired_vy[i],
                    kDtSec);
            t.log(Level.DEBUG, m_name, "max_vel_step", max_vel_step);

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
        t.log(Level.DEBUG, m_name, "s", min_s);
        return min_s;
    }

    @Override
    public String getGlassName() {
        return "DriveAccelerationLimiter";
    }

}
