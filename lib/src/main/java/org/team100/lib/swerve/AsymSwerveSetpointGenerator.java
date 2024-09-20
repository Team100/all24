package org.team100.lib.swerve;

import java.util.Optional;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.SupplierLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Just-in-time kinodynamic limits.
 * 
 * This version uses different limits for acceleration and for deceleration,
 * which better matches real robot behavior.
 * 
 * Takes a prior setpoint (ChassisSpeeds), a desired setpoint (from a driver, or
 * from a path follower), and outputs a new setpoint that respects all of the
 * kinematic constraints on module rotation speed and wheel velocity and
 * acceleration. By generating a new setpoint every iteration, the robot will
 * converge to the desired setpoint quickly while avoiding any intermediate
 * state that is kinematically infeasible (and can result in wheel slip or robot
 * heading drift as a result).
 */
public class AsymSwerveSetpointGenerator implements Glassy {
    // turns greater than this will flip
    // this used to be pi/2, which resulted in "square corner" paths
    private static final double flipLimitRad = 3 * Math.PI / 4;

    private final SwerveKinodynamics m_limits;

    private final CapsizeAccelerationLimiter m_centripetalLimiter;
    private final SteeringOverride m_SteeringOverride;
    private final SteeringRateLimiter m_steeringRateLimiter;
    private final DriveAccelerationLimiter m_DriveAccelerationLimiter;
    private final BatterySagLimiter m_BatterySagLimiter;

    public AsymSwerveSetpointGenerator(SupplierLogger parent, SwerveKinodynamics limits) {
        m_limits = limits;
        m_centripetalLimiter = new CapsizeAccelerationLimiter(parent, limits);
        m_SteeringOverride = new SteeringOverride(parent, limits);
        m_steeringRateLimiter = new SteeringRateLimiter(parent, limits);
        m_DriveAccelerationLimiter = new DriveAccelerationLimiter(parent, limits);
        m_BatterySagLimiter = new BatterySagLimiter();
    }

    /**
     * Generate a new setpoint.
     *
     * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the
     *                     previous iteration setpoint instead of the actual
     *                     measured/estimated kinematic state.
     * @param desiredState The desired state of motion, such as from the driver
     *                     sticks or a path following algorithm.
     * @param kDtSec       time in the future the setpoint should apply.
     * @return A Setpoint object that satisfies all of the KinematicLimits while
     *         converging to desiredState quickly.
     */
    public SwerveSetpoint generateSetpoint(
            SwerveSetpoint prevSetpoint,
            ChassisSpeeds desiredState,
            double kDtSec) {
        SwerveModuleState100[] prevModuleStates = prevSetpoint.getModuleStates();
        // the desired module state speeds are always positive.
        SwerveModuleState100[] desiredModuleStates;
        if (Experiments.instance.enabled(Experiment.UseSecondDerivativeSwerve)) {
            desiredModuleStates = m_limits.toSwerveModuleStatesWithoutDiscretization(
                    desiredState, prevSetpoint.getChassisSpeeds(), prevModuleStates, kDtSec);
        } else {
            desiredModuleStates = m_limits.toSwerveModuleStatesWithoutDiscretization(
                    desiredState);
        }
        desiredState = desaturate(desiredState, desiredModuleStates);
        boolean desiredIsStopped = SwerveUtil.desiredIsStopped(desiredState, desiredModuleStates, prevModuleStates);

        // For each module, compute local Vx and Vy vectors.
        double[] prev_vx = computeVx(prevModuleStates);
        double[] prev_vy = computeVy(prevModuleStates);
        // elements may be null.
        Rotation2d[] prev_heading = computeHeading(prevModuleStates);

        double[] desired_vx = computeVx(desiredModuleStates);
        double[] desired_vy = computeVy(desiredModuleStates);
        // elements may be null.
        Rotation2d[] desired_heading = computeHeading(desiredModuleStates);
        double[] desired_heading_velocity = computeHeadingVelocity(desiredModuleStates);

        boolean shouldStopAndReverse = shouldStopAndReverse(prev_heading, desired_heading);
        if (shouldStopAndReverse
                && !GeometryUtil.isZero(prevSetpoint.getChassisSpeeds())
                && !GeometryUtil.isZero(desiredState)) {
            // It will (likely) be faster to stop the robot, rotate the modules in place to
            // the complement of the desired angle, and accelerate again.
            return generateSetpoint(prevSetpoint, new ChassisSpeeds(), kDtSec);
        }

        // Compute the deltas between start and goal. We can then interpolate from the
        // start state to the goal state; then
        // find the amount we can move from start towards goal in this cycle such that
        // no kinematic limit is exceeded.

        ChassisSpeeds chassisSpeeds = prevSetpoint.getChassisSpeeds();
        double dx = desiredState.vxMetersPerSecond - chassisSpeeds.vxMetersPerSecond;
        double dy = desiredState.vyMetersPerSecond - chassisSpeeds.vyMetersPerSecond;
        double dtheta = desiredState.omegaRadiansPerSecond - chassisSpeeds.omegaRadiansPerSecond;

        // 's' interpolates between start and goal. At 0, we are at prevState and at 1,
        // we are at desiredState.

        double centripetal_min_s = m_centripetalLimiter.enforceCentripetalLimit(dx, dy, kDtSec);

        double min_s = centripetal_min_s;

        // In cases where an individual module is stopped, we want to remember the right
        // steering angle to command (since
        // inverse kinematics doesn't care about angle, we can be opportunistically
        // lazy).
        Rotation2d[] overrideSteering = new Rotation2d[prevModuleStates.length];

        if (desiredIsStopped) {
            for (int i = 0; i < prevModuleStates.length; ++i) {
                if (prevModuleStates[i].angle.isEmpty()) {
                    overrideSteering[i] = null;
                } else {
                    overrideSteering[i] = prevModuleStates[i].angle.get();
                }
            }
        } else {
            double override_min_s = m_SteeringOverride.overrideIfStopped(
                    desiredModuleStates,
                    prevModuleStates,
                    overrideSteering,
                    kDtSec);
            min_s = Math.min(min_s, override_min_s);

            double steering_min_s = m_steeringRateLimiter.enforceSteeringLimit(
                    prev_vx,
                    prev_vy,
                    prev_heading,
                    desired_vx,
                    desired_vy,
                    desired_heading,
                    desired_heading_velocity,
                    overrideSteering,
                    kDtSec);
            min_s = Math.min(min_s, steering_min_s);
        }

        double accel_min_s = m_DriveAccelerationLimiter.enforceWheelAccelLimit(
                prev_vx,
                prev_vy,
                desired_vx,
                desired_vy,
                kDtSec);

        min_s = Math.min(min_s, accel_min_s);

        if (Experiments.instance.enabled(Experiment.LimitBatterySag)) {
            double battery_min_s = m_BatterySagLimiter.get(RobotController.getBatteryVoltage());
            min_s = Math.min(min_s, battery_min_s);
        }

        return makeSetpoint(
                prevSetpoint,
                prevModuleStates,
                dx,
                dy,
                dtheta,
                min_s,
                overrideSteering,
                kDtSec);
    }

    @Override
    public String getGlassName() {
        return "AsymSwerveSetpointGenerator";
    }

    ///////////////////////////////////////////////////////

    private double[] computeVx(SwerveModuleState100[] states) {
        double[] vx = new double[states.length];
        for (int i = 0; i < states.length; ++i) {
            SwerveModuleState100 state = states[i];
            if (Math.abs(state.speedMetersPerSecond) < 1e-6 || state.angle.isEmpty()) {
                vx[i] = 0;
            } else {
                vx[i] = state.angle.get().getCos() * state.speedMetersPerSecond;
            }
        }
        return vx;
    }

    private double[] computeVy(SwerveModuleState100[] states) {
        double[] vy = new double[states.length];
        for (int i = 0; i < states.length; ++i) {
            SwerveModuleState100 state = states[i];
            if (Math.abs(state.speedMetersPerSecond) < 1e-6 || state.angle.isEmpty()) {
                vy[i] = 0;
            } else {
                vy[i] = state.angle.get().getSin() * state.speedMetersPerSecond;
            }
        }
        return vy;
    }

    /**
     * Which way each module is actually going, taking speed polarity into account.
     * 
     * @return elements are nullable.
     */
    private Rotation2d[] computeHeading(SwerveModuleState100[] states) {
        Rotation2d[] heading = new Rotation2d[states.length];
        for (int i = 0; i < states.length; ++i) {
            if (states[i].angle.isEmpty()) {
                heading[i] = null;
                continue;
            }
            heading[i] = states[i].angle.get();
            if (states[i].speedMetersPerSecond < 0.0) {
                heading[i] = GeometryUtil.flip(heading[i]);
            }
        }
        return heading;
    }

    /**
     * Which way each module is actually going, taking speed polarity into account.
     */
    private double[] computeHeadingVelocity(SwerveModuleState100[] states) {
        double[] heading = new double[states.length];
        for (int i = 0; i < states.length; ++i) {
            heading[i] = states[i].omega;
        }
        return heading;
    }

    /**
     * If we want to go back the way we came, it might be faster to stop
     * and then reverse. This is certainly true for near-180 degree turns, but
     * it's definitely not true for near-90 degree turns.
     */
    private boolean shouldStopAndReverse(Rotation2d[] prev_heading, Rotation2d[] desired_heading) {
        for (int i = 0; i < prev_heading.length; ++i) {
            if (desired_heading[i] == null || prev_heading[i] == null) {
                return false;
            }
            Rotation2d diff = desired_heading[i].minus(prev_heading[i]);
            if (Math.abs(diff.getRadians()) < flipLimitRad) {
                return false;
            }
        }
        return true;
    }

    /**
     * Make sure desiredState respects velocity limits.
     * 
     * does not involve the Tire model.
     */
    private ChassisSpeeds desaturate(
            ChassisSpeeds desiredState,
            SwerveModuleState100[] desiredModuleStates) {
        if (m_limits.getMaxDriveVelocityM_S() > 0.0) {
            SwerveDriveKinematics100.desaturateWheelSpeeds(desiredModuleStates, m_limits.getMaxDriveVelocityM_S());
            desiredState = m_limits.toChassisSpeeds(desiredModuleStates);
        }
        return desiredState;
    }

    private SwerveSetpoint makeSetpoint(
            final SwerveSetpoint prevSetpoint,
            SwerveModuleState100[] prevModuleStates,
            double dx,
            double dy,
            double dtheta,
            double min_s,
            Rotation2d[] overrideSteering,
            double kDtSec) {
        ChassisSpeeds setpointSpeeds = makeSpeeds(
                prevSetpoint.getChassisSpeeds(),
                dx,
                dy,
                dtheta,
                min_s,
                kDtSec);
        SwerveModuleState100[] setpointStates;
        // the speeds in these states are always positive.
        if (Experiments.instance.enabled(Experiment.UseSecondDerivativeSwerve)) {
            setpointStates = m_limits.toSwerveModuleStates(
                    setpointSpeeds,
                    prevSetpoint.getChassisSpeeds(),
                    prevModuleStates,
                    setpointSpeeds.omegaRadiansPerSecond,
                    kDtSec);
        } else {
            setpointStates = m_limits.toSwerveModuleStates(
                    setpointSpeeds,
                    setpointSpeeds.omegaRadiansPerSecond,
                    kDtSec);
        }

        applyOverrides(overrideSteering, setpointStates);
        flipIfRequired(prevModuleStates, setpointStates);

        return new SwerveSetpoint(setpointSpeeds, setpointStates);
    }

    /** Overwrite the states with the supplied steering overrides, if any. */
    private void applyOverrides(Rotation2d[] overrides, SwerveModuleState100[] states) {
        for (int i = 0; i < states.length; ++i) {
            if (states[i].angle.isEmpty()) {
                continue;
            }
            final Rotation2d maybeOverride = overrides[i];
            if (maybeOverride != null) {
                Rotation2d override = maybeOverride;
                if (SwerveUtil.shouldFlip(override.minus(states[i].angle.get()))) {
                    states[i].speedMetersPerSecond *= -1.0;
                }
                states[i].angle = Optional.of(override);
            }
        }
    }

    private void flipIfRequired(SwerveModuleState100[] prevStates, SwerveModuleState100[] setpointStates) {
        for (int i = 0; i < prevStates.length; ++i) {
            if (setpointStates[i].angle.isEmpty() || prevStates[i].angle.isEmpty()) {
                continue;
            }
            final Rotation2d deltaRotation = setpointStates[i].angle.get().minus(prevStates[i].angle.get());
            if (SwerveUtil.shouldFlip(deltaRotation)) {
                setpointStates[i].angle = Optional.of(GeometryUtil.flip(setpointStates[i].angle.get()));
                setpointStates[i].speedMetersPerSecond *= -1.0;
                setpointStates[i].accelMetersPerSecond_2 *= -1.0;
            }
        }
    }

    /**
     * Applies two transforms to the previous speed:
     * 
     * 1. Scales the commanded accelerations by min_s, i.e. applies the constraints
     * calculated earlier.
     * 
     * 2. Transforms translations according to the rotational velocity, regardless
     * of
     * min_s -- essentially modeling inertia. This part was missing before, which I
     * think must just be a mistake.
     */
    private static ChassisSpeeds makeSpeeds(
            ChassisSpeeds prev,
            double dx,
            double dy,
            double dtheta,
            double min_s,
            double kDtSec) {
        double omega = prev.omegaRadiansPerSecond + min_s * dtheta;
        double drift = -1.0 * omega * kDtSec;
        double vx = prev.vxMetersPerSecond * Math.cos(drift)
                - prev.vyMetersPerSecond * Math.sin(drift)
                + min_s * dx;
        double vy = prev.vxMetersPerSecond * Math.sin(drift)
                + prev.vyMetersPerSecond * Math.cos(drift)
                + min_s * dy;
        return new ChassisSpeeds(vx, vy, omega);
    }
}
