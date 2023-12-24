package org.team100.lib.timing;

import java.util.Optional;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.swerve.SwerveKinematicLimits;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Note this does not handle centripetal acceleration or steering rate.
 * 
 * TODO: maybe it's not worth keeping?
 */
public class SwerveDriveDynamicsConstraint implements TimingConstraint {
    private final SwerveDriveKinematics m_kinematics;
    private final SwerveKinematicLimits m_limits;

    public SwerveDriveDynamicsConstraint(SwerveDriveKinematics kinematics, SwerveKinematicLimits limits) {
        m_kinematics = kinematics;
        m_limits = limits;
    }

    /**
     * Given a target spatial heading rate (rad/m), return the maximum translational
     * speed allowed (m/s) that maintains the target spatial heading rate.
     * 
     * Note the setpoint generator respects acceleration limits so this is likely to
     * do the wrong thing, if the "entry speed" (1m/s) is faster than the allowed
     * speed.
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

        SwerveModuleState[] module_states = m_kinematics.toSwerveModuleStates(chassis_speeds);
        double max_vel = Double.POSITIVE_INFINITY;
        for (var module : module_states) {
            max_vel = Math.min(max_vel, m_limits.kMaxDriveVelocity / Math.abs(module.speedMetersPerSecond));
        }
        return max_vel;
    }

    /**
     * Simply provide the configured drive acceleration limits.
     */
    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocity) {
        return new MinMaxAcceleration(
                -m_limits.kMaxDriveDeceleration,
                m_limits.kMaxDriveAcceleration);
    }
}