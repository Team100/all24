package org.team100.lib.swerve;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

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

    private final SwerveKinodynamics m_limits;
    // LOGGER
    private final DoubleSupplierLogger2 m_log_s;

    public SteeringRateLimiter(SupplierLogger2 parent, SwerveKinodynamics limits) {
        SupplierLogger2 child = parent.child(this);
        m_limits = limits;
        m_log_s = child.doubleLogger(Level.TRACE, "s");
    }

    public double enforceSteeringLimit(
            double[] prev_vx,
            double[] prev_vy,
            Rotation2d[] prev_heading, // nullable entries
            double[] desired_vx,
            double[] desired_vy,
            Rotation2d[] desired_heading, // nullable entries
            double[] desired_heading_velocity,
            Rotation2d[] overrideSteering) {

        double min_s = 1.0;

        for (int i = 0; i < prev_vx.length; ++i) {
            if (prev_heading[i] == null || desired_heading[i] == null) {
                // don't know what to do here
                continue;
            }
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
                    desired_heading_velocity[i] * TimedRobot100.LOOP_PERIOD_S,
                    TimedRobot100.LOOP_PERIOD_S * m_limits.getMaxSteeringVelocityRad_S(),
                    kMaxIterations);
            } else {
                s = SwerveUtil.findSteeringMaxS(
                        prev_vx[i],
                        prev_vy[i],
                        prev_heading[i].getRadians(),
                        desired_vx[i],
                        desired_vy[i],
                        desired_heading[i].getRadians(),
                        TimedRobot100.LOOP_PERIOD_S * m_limits.getMaxSteeringVelocityRad_S(),
                        kMaxIterations);
            }
            min_s = Math.min(min_s, s);
        }
        double s = min_s;
        m_log_s.log( () -> s);
        return min_s;
    }

}
