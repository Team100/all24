package org.team100.lib.swerve;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Enforces steering velocity limits.
 * 
 * Takes the derivative of steering angle at the current angle, and then backs
 * out the maximum interpolant between start and goal states. Remembers the
 * minimum across all modules, since that is the active constraint.
 */
public class SteeringRateLimiter implements Glassy {
    private static final int kMaxIterations = 10;

    private final Logger m_logger;
    private final SwerveKinodynamics m_limits;

    public SteeringRateLimiter(Logger parent, SwerveKinodynamics limits) {
        m_logger = parent.child(this);
        m_limits = limits;
    }

    public double enforceSteeringLimit(
            double[] prev_vx,
            double[] prev_vy,
            Rotation2d[] prev_heading,
            double[] desired_vx,
            double[] desired_vy,
            Rotation2d[] desired_heading,
            double[] desired_heading_velocity,
            Rotation2d[] overrideSteering,
            double kDtSec) {

        double min_s = 1.0;

        for (int i = 0; i < prev_vx.length; ++i) {
            if (overrideSteering[i] != null) {
                // ignore overridden wheels
                continue;
            }
            double s;
            if (Experiments.instance.enabled(Experiment.UseSecondDerivativeSwerve)) {
                s = SwerveUtil.findSteeringMaxS(
                    prev_vx[i],
                    prev_vy[i],
                    prev_heading[i].getRadians(),
                    desired_vx[i],
                    desired_vy[i],
                    desired_heading[i].getRadians(),
                    desired_heading_velocity[i] * kDtSec,
                    kDtSec * m_limits.getMaxSteeringVelocityRad_S(),
                    kMaxIterations);
            } else {
                s = SwerveUtil.findSteeringMaxS(
                        prev_vx[i],
                        prev_vy[i],
                        prev_heading[i].getRadians(),
                        desired_vx[i],
                        desired_vy[i],
                        desired_heading[i].getRadians(),
                        kDtSec * m_limits.getMaxSteeringVelocityRad_S(),
                        kMaxIterations);
            }
            min_s = Math.min(min_s, s);
        }
        double s = min_s;
        m_logger.logDouble(Level.TRACE, "s", () -> s);
        return min_s;
    }

    @Override
    public String getGlassName() {
        return "SteeringRateLimiter";
    }

}
