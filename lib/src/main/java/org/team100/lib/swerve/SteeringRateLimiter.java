package org.team100.lib.swerve;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Enforces steering velocity limits.
 * 
 * Takes the derivative of steering angle at the current angle, and then backs
 * out the maximum interpolant between start and goal states. Remembers the
 * minimum across all modules, since that is the active constraint.
 */
public class SteeringRateLimiter implements Glassy {
    private static final Telemetry t = Telemetry.get();
    private static final int kMaxIterations = 10;
    private final SwerveKinodynamics m_limits;
    private final String m_name;

    public SteeringRateLimiter(String parent, SwerveKinodynamics limits) {
        m_name = Names.append(parent, this);
        m_limits = limits;
    }

    /** note overrideSteering is an outvar */
    public double enforceSteeringLimit(
            SwerveModuleState[] desiredModuleStates,
            SwerveModuleState[] prevModuleStates,
            double[] prev_vx,
            double[] prev_vy,
            Rotation2d[] prev_heading,
            double[] desired_vx,
            double[] desired_vy,
            Rotation2d[] desired_heading,
            Rotation2d[] overrideSteering,
            double kDtSec) {

        double min_s = 1.0;
        final double max_theta_step = kDtSec * m_limits.getMaxSteeringVelocityRad_S();
        for (int i = 0; i < prevModuleStates.length; ++i) {
            if (Math.abs(prevModuleStates[i].speedMetersPerSecond - 0.0) <= 1e-12) {
                // If module is stopped, we know that we will need to move straight to the final
                // steering angle, so limit based purely on rotation in place.

                if (Math.abs(desiredModuleStates[i].speedMetersPerSecond - 0.0) <= 1e-12) {
                    // Both previous and desired states are stopped.
                    // Just leave module at its current angle.
                    overrideSteering[i] = prevModuleStates[i].angle;
                    continue;
                }

                Rotation2d necessaryRotation = desiredModuleStates[i].angle.minus(prevModuleStates[i].angle);
                if (SwerveUtil.shouldFlip(necessaryRotation)) {
                    necessaryRotation = necessaryRotation.rotateBy(GeometryUtil.kRotation180);
                }
                double rotationRad = MathUtil.angleModulus(necessaryRotation.getRadians());
                final double numStepsNeeded = Math.abs(rotationRad) / max_theta_step;

                if (numStepsNeeded <= 1.0) {
                    // Steer directly to goal angle.
                    overrideSteering[i] = desiredModuleStates[i].angle;
                    // Don't limit the global min_s;
                    continue;
                } else {
                    // Adjust steering by max_theta_step.
                    overrideSteering[i] =  prevModuleStates[i].angle.rotateBy(
                            Rotation2d.fromRadians(Math.signum(rotationRad) * max_theta_step));
                    min_s = 0.0;
                    continue;
                }
            }
            if (min_s == 0.0) {
                // s can't get any lower. Save some CPU.
                continue;
            }

            double s = SwerveUtil.findSteeringMaxS(
                    prev_vx[i],
                    prev_vy[i],
                    prev_heading[i].getRadians(),
                    desired_vx[i],
                    desired_vy[i],
                    desired_heading[i].getRadians(),
                    max_theta_step,
                    kMaxIterations);
            min_s = Math.min(min_s, s);
        }
        t.log(Level.DEBUG, m_name, "s", min_s);
        return min_s;
    }

    @Override
    public String getGlassName() {
        return "SteeringRateLimiter";
    }

}
