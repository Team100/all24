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
 * if any module is stopped and misaligned, stop everything until it's aligned.
 */
public class SteeringOverride implements Glassy {
    private static final double kEpsilon = 1e-3;

    private final Telemetry.Logger t;
    private final SwerveKinodynamics m_limits;
    private final String m_name;

    public SteeringOverride(String parent, SwerveKinodynamics limits) {
        m_name = Names.append(parent, this);
        t = Telemetry.get().logger(m_name);
        m_limits = limits;
    }

    public double overrideIfStopped(
            SwerveModuleState[] desiredModuleStates,
            SwerveModuleState[] prevModuleStates,
            Rotation2d[] overrideSteering,
            double kDtSec) {
        // in one cycle we can go this many radians. note this assumes infinite
        // acceleration; if the steering axes are slow to accelerate, maybe change this?
        double maxThetaStepRad = kDtSec * m_limits.getMaxSteeringVelocityRad_S();
        double min_s = 1.0;
        for (int i = 0; i < prevModuleStates.length; ++i) {
            if (Math.abs(prevModuleStates[i].speedMetersPerSecond) <= kEpsilon) {
                // If module is stopped, we know that we will need to move straight to the final
                // steering angle, so limit based purely on rotation in place.
                if (Math.abs(desiredModuleStates[i].speedMetersPerSecond) <= kEpsilon) {
                    // Both previous and desired states are stopped.
                    // Just leave module at its current angle.
                    overrideSteering[i] = prevModuleStates[i].angle;
                    continue;
                }

                double rotationRad = rotationRad(desiredModuleStates[i], prevModuleStates[i]);

                double numStepsNeeded = Math.abs(rotationRad) / maxThetaStepRad;

                if (numStepsNeeded <= 1.0) {
                    // goal is achievable in one time step.
                    // note this angle is the *unflipped* one, which means that something downstream
                    // may decide to flip it.
                    overrideSteering[i] = desiredModuleStates[i].angle;
                } else {
                    // goal is not achievable, so move as much as possible in one step.
                    // note this moves in the "flipped" direction if required.
                    Rotation2d oneStepOfRotation = Rotation2d.fromRadians(Math.signum(rotationRad) * maxThetaStepRad);
                    overrideSteering[i] = prevModuleStates[i].angle.rotateBy(oneStepOfRotation);
                    // stop all drive motors until steering is aligned
                    min_s = 0.0;
                }
            }
        }
        t.log(Level.DEBUG, "s", min_s);
        return min_s;
    }

    @Override
    public String getGlassName() {
        return "SteeringOverride";
    }

    /** Actual rotation required, taking flipping into account. */
    private double rotationRad(SwerveModuleState desiredModuleState, SwerveModuleState prevModuleState) {
        Rotation2d necessaryRotation = desiredModuleState.angle.minus(prevModuleState.angle);
        if (SwerveUtil.shouldFlip(necessaryRotation)) {
            necessaryRotation = necessaryRotation.rotateBy(GeometryUtil.kRotation180);
        }
        return MathUtil.angleModulus(necessaryRotation.getRadians());
    }
}
