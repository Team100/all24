package org.team100.lib.timing;

import java.util.Optional;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.swerve.SwerveSetpoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Apply setpoint generator constraints to paths, so that the setpoint generator
 * won't have to do anything in the follower.
 */
public class SwerveSetpointGeneratorConstraint implements TimingConstraint {
    private static final double kDt = 0.02;
    private final SwerveDriveKinematics m_kinematics;
    private final AsymSwerveSetpointGenerator.KinematicLimits m_limits;

    private final AsymSwerveSetpointGenerator setpoint_generator_;

    public SwerveSetpointGeneratorConstraint(
            SwerveDriveKinematics kinematics,
            AsymSwerveSetpointGenerator.KinematicLimits limits) {
        m_kinematics = kinematics;
        m_limits = limits;

        setpoint_generator_ = new AsymSwerveSetpointGenerator(kinematics);
    }

    @Override
    public double getMaxVelocity(Pose2dWithMotion state) {

        Optional<Rotation2d> course = state.getCourse();
        Rotation2d course_local = state.getHeading().unaryMinus()
                .rotateBy(course.isPresent() ? course.get() : GeometryUtil.kRotationZero);
        double vx = course_local.getCos();
        double vy = course_local.getSin();
        // rad/m
        double vtheta = state.getHeadingRate();
        double curvature = state.getCurvature();

        // this is a "spatial speed", direction and rad/m
        ChassisSpeeds spatialSpeed = new ChassisSpeeds(vx, vy, vtheta);

        SwerveModuleState[] module_states = m_kinematics.toSwerveModuleStates(spatialSpeed);
        double max_vel = Double.POSITIVE_INFINITY;
        for (var module : module_states) {
            max_vel = Math.min(max_vel, m_limits.kMaxDriveVelocity / Math.abs(module.speedMetersPerSecond));
        }

        // this is an actual speed m/s and rad/s.
        ChassisSpeeds maxSpeed = new ChassisSpeeds(
                vx * max_vel,
                vy * max_vel,
                vtheta * max_vel);

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(maxSpeed);

        SwerveSetpoint setpoint = new SwerveSetpoint(maxSpeed, states);

        // correct the course for curvature
        course_local = course_local.rotateBy(Rotation2d.fromRadians(kDt * max_vel * curvature));
        // correct the course for heading rate
        course_local = course_local.rotateBy(Rotation2d.fromRadians(-kDt * max_vel * vtheta));
        ChassisSpeeds nextSpatialSpeed = new ChassisSpeeds(
                course_local.getCos(),
                course_local.getSin(),
                vtheta);
        SwerveSetpoint nextSetpoint = setpoint_generator_.generateSetpoint(
                m_limits,
                setpoint,
                nextSpatialSpeed,
                kDt);
        double nextSetpointSpeed = GeometryUtil.norm(GeometryUtil.toTwist2d(nextSetpoint.getChassisSpeeds()));
        double ratio = nextSetpointSpeed /
                GeometryUtil.norm(GeometryUtil.toTwist2d(nextSpatialSpeed));
        if (ratio > 0.0 && ratio < 1) {
            max_vel *= ratio;
        }

        return max_vel;
    }

    /**
     * Blindly applies the configuration.
     */
    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocityM_S) {
        return new MinMaxAcceleration(
                -m_limits.kMaxDriveDecceleration,
                m_limits.kMaxDriveAcceleration);
    }

}
