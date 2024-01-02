package org.team100.lib.timing;

import java.util.Optional;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Linear velocity limit based on spatial yaw rate and drive wheel speed limit.
 * 
 * Slows the path velocity to accommodate the desired yaw rate.
 * 
 * This *should* provide the same answer as the YawRateConstraint, if the
 * omega limit calculation is correct.
 */
public class SwerveDriveDynamicsConstraint implements TimingConstraint {
    /** Maybe this would be better passed in to the constructor. */
    private static final double kDtSec = 0.02;
    private final SwerveKinodynamics m_limits;

    public SwerveDriveDynamicsConstraint(SwerveKinodynamics limits) {
        m_limits = limits;
    }

    /**
     * Given a target spatial heading rate (rad/m), return the maximum translational
     * speed allowed (m/s) that maintains the target spatial heading rate.
     */
    @Override
    public double getMaxVelocity(Pose2dWithMotion state) {
        // First check instantaneous velocity and compute a limit based on drive
        // velocity.
        Optional<Rotation2d> course = state.getCourse();
        Rotation2d course_local = state.getHeading().unaryMinus()
                .rotateBy(course.isPresent() ? course.get() : GeometryUtil.kRotationZero);
        double vx = course_local.getCos();
        double vy = course_local.getSin();
        // rad/m
        double vtheta = state.getHeadingRate();

        // first compute the effect of heading rate

        // this is a "spatial speed," direction and rad/m
        // which is like moving 1 m/s.
        ChassisSpeeds chassis_speeds = new ChassisSpeeds(vx, vy, vtheta);

        SwerveModuleState[] module_states = m_limits.toSwerveModuleStates(chassis_speeds, kDtSec);
        double max_vel = Double.POSITIVE_INFINITY;
        for (var module : module_states) {
            max_vel = Math.min(max_vel, m_limits.getMaxDriveVelocityM_S() / Math.abs(module.speedMetersPerSecond));
        }
        return max_vel;
    }

    /**
     * Simply provide the configured drive acceleration limits.
     */
    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocity) {
        return new MinMaxAcceleration(
                -m_limits.getMaxDriveDecelerationM_S2(),
                m_limits.getMaxDriveAccelerationM_S2());
    }
}