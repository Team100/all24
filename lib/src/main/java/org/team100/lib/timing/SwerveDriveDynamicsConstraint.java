package org.team100.lib.timing;

import java.util.Optional;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.swerve.SwerveKinematicLimits;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * This is based on 254 2023 version.
 */
public class SwerveDriveDynamicsConstraint implements TimingConstraint {
    private final SwerveDriveKinematics m_kinematics;
    private final SwerveKinematicLimits m_limits;
    private final AsymSwerveSetpointGenerator setpoint_generator_;

    public SwerveDriveDynamicsConstraint(SwerveDriveKinematics kinematics,
            SwerveKinematicLimits limits) {
        m_kinematics = kinematics;
        m_limits = limits;
        setpoint_generator_ = new AsymSwerveSetpointGenerator(kinematics);
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
        // double curvature = state.getCurvature();

        // this is a "speed" based on the course only.
        ChassisSpeeds chassis_speeds = new ChassisSpeeds(vx, vy, vtheta);

        SwerveModuleState[] module_states = m_kinematics.toSwerveModuleStates(chassis_speeds);
        double max_vel = Double.POSITIVE_INFINITY;
        for (var module : module_states) {
            max_vel = Math.min(max_vel, m_limits.kMaxDriveVelocity / Math.abs(module.speedMetersPerSecond));
        }
        /*
         * chassis_speeds = new ChassisSpeeds(vx * max_vel, vy * max_vel, vtheta *
         * max_vel);
         * var setpoint = new SwerveSetpoint(new ChassisSpeeds(), new
         * SwerveModuleState[kinematics_.getNumModules()]);
         * var states = kinematics_.toSwerveModuleStates(chassis_speeds);
         * setpoint.mChassisSpeeds = chassis_speeds;
         * setpoint.mModuleStates = states;
         * 
         * final double kDt = 0.001;
         * course_local.rotateBy(Rotation2d.fromRadians(kDt * max_vel * curvature));
         * course_local.rotateBy(Rotation2d.fromRadians(-kDt * max_vel * vtheta));
         * var next_chassis_speeds = new ChassisSpeeds(course_local.cos(),
         * course_local.sin(), vtheta);
         * var next_setpoint = setpoint_generator_.generateSetpoint(limits_, setpoint,
         * next_chassis_speeds, kDt);
         * double scale_factor = next_setpoint.mChassisSpeeds.toTwist2d().norm2() /
         * next_chassis_speeds.toTwist2d().norm2();
         * if (scale_factor > 0.0) {
         * max_vel *= scale_factor;
         * }
         */

        /*
         * // Next check curvature and heading rate to back out steering
         * velocity-limited max vel.
         * // To do this, we need to compute the rate of change of steering angle as a
         * function of our
         * // curvature and rate of change of heading. Steering angle is ultimately
         * based on calling atan2
         * // (the angle function) on the local x and y velocities. Its derivative is
         * given by:
         * // dAtan2(y(t),x(t))/dt = x/(x^2+y^2)*dy/dt - y/(x^2+y^2)*dx/dt
         * var module_locations = kinematics_.getModuleLocations();
         * for (var location : module_locations) {
         * // Numerators (instantaneous velocities)
         * double x = vx + location.x() * vtheta; // * max_vel
         * double y = vy - location.y() * vtheta; // * max_vel
         * // Denominator (total velocity squared)
         * double x2y2 = x*x + y*y; // * 2*max_vel^2
         * 
         * // Accelerations
         * double dx = -course_local.sin() * (curvature + vtheta * location.x());
         * double dy = course_local.cos() * (curvature - vtheta * location.y());
         * 
         * double steering_rate = (x * dy - y * dx) / x2y2;
         * max_vel = Math.min(max_vel, Math.sqrt(steering_rate));
         * }
         */

        return max_vel;
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocity) {
        // Just check drive acceleration limits.
        return new MinMaxAcceleration(-m_limits.kMaxDriveAcceleration, m_limits.kMaxDriveAcceleration);
    }
}