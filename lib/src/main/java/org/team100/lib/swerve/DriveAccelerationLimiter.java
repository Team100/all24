package org.team100.lib.swerve;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import edu.wpi.first.math.kinematics.SwerveModuleState;

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
            SwerveModuleState[] prevModuleStates,
            double[] prev_vx,
            double[] prev_vy,
            double[] desired_vx,
            double[] desired_vy,
            double kDtSec) {
        double min_s = 1.0;
        // Enforce drive wheel acceleration limits.
        for (int i = 0; i < prevModuleStates.length; ++i) {
            if (min_s == 0.0) {
                // No need to carry on.
                break;
            }

            double max_vel_step = SwerveUtil.getMaxVelStep(
                    m_limits,
                    prev_vx[i],
                    prev_vy[i],
                    desired_vx[i],
                    desired_vy[i],
                    kDtSec);
            t.log(Level.DEBUG, m_name, "max_vel_step", max_vel_step);

            double vx_min_s = min_s == 1.0 ? desired_vx[i] : (desired_vx[i] - prev_vx[i]) * min_s + prev_vx[i];
            double vy_min_s = min_s == 1.0 ? desired_vy[i] : (desired_vy[i] - prev_vy[i]) * min_s + prev_vy[i];
            // Find the max s for this drive wheel. Search on the interval between 0 and
            // min_s, because we already know we can't go faster than that.
            double s = min_s * SwerveUtil.findDriveMaxS(
                    prev_vx[i],
                    prev_vy[i],
                    Math.hypot(prev_vx[i], prev_vy[i]),
                    vx_min_s,
                    vy_min_s,
                    Math.hypot(vx_min_s, vy_min_s),
                    max_vel_step,
                    kMaxIterations);
            min_s = Math.min(min_s, s);
        }
        t.log(Level.DEBUG, m_name, "s", min_s);
        return min_s;
    }

    @Override
    public String getGlassName() {
        return "DriveAccelerationLimiter";
    }

    
    
}
