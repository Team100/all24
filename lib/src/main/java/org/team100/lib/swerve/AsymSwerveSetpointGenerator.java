package org.team100.lib.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Names;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
    private static final double flipLimit = 3 * Math.PI / 4;

    private final SwerveKinodynamics m_limits;

    private final CapsizeAccelerationLimiter m_centripetalLimiter;
    private final SteeringRateLimiter m_steeringRateLimiter;
    private final DriveAccelerationLimiter m_DriveAccelerationLimiter;
    private final String m_name;

    public AsymSwerveSetpointGenerator(String parent, SwerveKinodynamics limits) {
        m_limits = limits;
        m_name = Names.append(parent, this);
        m_centripetalLimiter = new CapsizeAccelerationLimiter(m_name, limits);
        m_steeringRateLimiter = new SteeringRateLimiter(m_name, limits);
        m_DriveAccelerationLimiter = new DriveAccelerationLimiter(m_name, limits);
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
        SwerveModuleState[] desiredModuleStates = m_limits.toSwerveModuleStatesWithoutDiscretization(
                desiredState);
        desiredState = desaturate(desiredState, desiredModuleStates);
        SwerveModuleState[] prevModuleStates = prevSetpoint.getModuleStates();
        boolean need_to_steer = SwerveUtil.makeStop(desiredState, desiredModuleStates, prevModuleStates);

        // For each module, compute local Vx and Vy vectors.
        double[] prev_vx = new double[prevModuleStates.length];
        double[] prev_vy = new double[prevModuleStates.length];
        Rotation2d[] prev_heading = new Rotation2d[prevModuleStates.length];

        double[] desired_vx = new double[prevModuleStates.length];
        double[] desired_vy = new double[prevModuleStates.length];
        Rotation2d[] desired_heading = new Rotation2d[prevModuleStates.length];

        boolean all_modules_should_flip = maybeFlip(
                desiredModuleStates,
                prevModuleStates,
                prev_vx,
                prev_vy,
                prev_heading,
                desired_vx,
                desired_vy,
                desired_heading);

        if (all_modules_should_flip &&
                !GeometryUtil.toTwist2d(prevSetpoint.getChassisSpeeds()).equals(GeometryUtil.kTwist2dIdentity) &&
                !GeometryUtil.toTwist2d(desiredState).equals(GeometryUtil.kTwist2dIdentity)) {
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
        List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(prevModuleStates.length);
        double steering_min_s = m_steeringRateLimiter.enforceSteeringLimit(
                desiredModuleStates,
                prevModuleStates,
                need_to_steer,
                prev_vx,
                prev_vy,
                prev_heading,
                desired_vx,
                desired_vy,
                desired_heading,
                overrideSteering,
                kDtSec);

        min_s = Math.min(min_s, steering_min_s);

        double accel_min_s = m_DriveAccelerationLimiter.enforceWheelAccelLimit(
                prevModuleStates,
                prev_vx,
                prev_vy,
                desired_vx,
                desired_vy,
                kDtSec);

        min_s = Math.min(min_s, accel_min_s);

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

    /**
     * If we want to go back the way we came, it might be faster to stop
     * and then reverse. This is certainly true for near-180 degree turns, but
     * it's definitely not true for near-90 degree turns.
     */
    private boolean maybeFlip(
            SwerveModuleState[] desiredModuleStates,
            SwerveModuleState[] prevModuleStates,
            double[] prev_vx,
            double[] prev_vy,
            Rotation2d[] prev_heading,
            double[] desired_vx,
            double[] desired_vy,
            Rotation2d[] desired_heading) {
        boolean all_modules_should_flip = true;
        for (int i = 0; i < prevModuleStates.length; ++i) {
            prev_vx[i] = prevModuleStates[i].angle.getCos() * prevModuleStates[i].speedMetersPerSecond;
            prev_vy[i] = prevModuleStates[i].angle.getSin() * prevModuleStates[i].speedMetersPerSecond;
            prev_heading[i] = prevModuleStates[i].angle;
            if (prevModuleStates[i].speedMetersPerSecond < 0.0) {
                prev_heading[i] = GeometryUtil.flip(prev_heading[i]);
            }

            desired_vx[i] = desiredModuleStates[i].angle.getCos() * desiredModuleStates[i].speedMetersPerSecond;
            desired_vy[i] = desiredModuleStates[i].angle.getSin() * desiredModuleStates[i].speedMetersPerSecond;
            desired_heading[i] = desiredModuleStates[i].angle;

            if (desiredModuleStates[i].speedMetersPerSecond < 0.0) {
                desired_heading[i] = GeometryUtil.flip(desired_heading[i]);
            }
            if (all_modules_should_flip) {
                double required_rotation_rad = Math
                        .abs(prev_heading[i].unaryMinus().rotateBy(desired_heading[i]).getRadians());
                if (required_rotation_rad < flipLimit) {
                    all_modules_should_flip = false;
                }
            }
        }
        return all_modules_should_flip;
    }

    /**
     * Make sure desiredState respects velocity limits.
     */
    private ChassisSpeeds desaturate(
            ChassisSpeeds desiredState,
            SwerveModuleState[] desiredModuleStates) {
        if (m_limits.getMaxDriveVelocityM_S() > 0.0) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, m_limits.getMaxDriveVelocityM_S());
            desiredState = m_limits.toChassisSpeeds(desiredModuleStates);
        }
        return desiredState;
    }

    private SwerveSetpoint makeSetpoint(
            final SwerveSetpoint prevSetpoint,
            SwerveModuleState[] prevModuleStates,
            double dx,
            double dy,
            double dtheta,
            double min_s,
            List<Optional<Rotation2d>> overrideSteering,
            double kDtSec) {
        ChassisSpeeds retSpeeds = makeSpeeds(
                prevSetpoint.getChassisSpeeds(),
                dx,
                dy,
                dtheta,
                min_s,
                kDtSec);

        SwerveModuleState[] retStates = m_limits.toSwerveModuleStates(
                retSpeeds,
                retSpeeds.omegaRadiansPerSecond,
                kDtSec);

        flipIfRequired(prevModuleStates, overrideSteering, retStates);
        return new SwerveSetpoint(retSpeeds, retStates);
    }

    private void flipIfRequired(
            SwerveModuleState[] prevModuleStates,
            List<Optional<Rotation2d>> overrideSteering,
            SwerveModuleState[] retStates) {
        for (int i = 0; i < prevModuleStates.length; ++i) {
            final Optional<Rotation2d> maybeOverride = overrideSteering.get(i);
            if (maybeOverride.isPresent()) {
                Rotation2d override = maybeOverride.get();
                if (SwerveUtil.flipHeading(retStates[i].angle.unaryMinus().rotateBy(override))) {
                    retStates[i].speedMetersPerSecond *= -1.0;
                }
                retStates[i].angle = override;
            }
            final Rotation2d deltaRotation = prevModuleStates[i].angle.unaryMinus().rotateBy(retStates[i].angle);
            if (SwerveUtil.flipHeading(deltaRotation)) {
                retStates[i].angle = GeometryUtil.flip(retStates[i].angle);
                retStates[i].speedMetersPerSecond *= -1.0;
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
        double drift = -omega * kDtSec;
        double vx = prev.vxMetersPerSecond * Math.cos(drift)
                - prev.vyMetersPerSecond * Math.sin(drift)
                + min_s * dx;
        double vy = prev.vxMetersPerSecond * Math.sin(drift)
                + prev.vyMetersPerSecond * Math.cos(drift)
                + min_s * dy;
        return new ChassisSpeeds(
                vx,
                vy,
                omega);
    }
}
