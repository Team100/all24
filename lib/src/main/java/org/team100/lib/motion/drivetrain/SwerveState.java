package org.team100.lib.motion.drivetrain;

import java.util.Optional;

import org.team100.lib.controller.State100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;

/**
 * Describes the state of a holonomic drive in three dimensions,
 * x, y, and theta, each of which is represented by position, velocity,
 * and acceleration.
 */
public class SwerveState {
    private final State100 m_x;
    private final State100 m_y;
    private final State100 m_theta;

    public SwerveState(State100 x, State100 y, State100 theta) {
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

    public SwerveState(Pose2d x, FieldRelativeVelocity v) {
        this(
                new State100(x.getX(), v.x(), 0),
                new State100(x.getY(), v.y(), 0),
                new State100(x.getRotation().getRadians(), v.theta(), 0));
    }

    public SwerveState(Pose2d x, FieldRelativeVelocity v, FieldRelativeAcceleration a) {
        this(
                new State100(x.getX(), v.x(), a.x()),
                new State100(x.getY(), v.y(), a.y()),
                new State100(x.getRotation().getRadians(), v.theta(), a.theta()));
    }

    public SwerveState() {
        this(new State100(), new State100(), new State100());
    }

    public Pose2d pose() {
        return new Pose2d(m_x.x(), m_y.x(), new Rotation2d(m_theta.x()));
    }

    /** Translation of the pose */
    public Translation2d translation() {
        return new Translation2d(m_x.x(), m_y.x());
    }

    public FieldRelativeVelocity velocity() {
        return new FieldRelativeVelocity(m_x.v(), m_y.v(), m_theta.v());
    }

    public State100 x() {
        return m_x;
    }

    public State100 y() {
        return m_y;
    }

    public State100 theta() {
        return m_theta;
    }

    /**
     * turn a wpi trajectory state into a swervestate.
     * 
     * does not account for centripetal acceleration.
     */
    public static SwerveState fromState(State desiredState, Rotation2d desiredRot) {
        double xx = desiredState.poseMeters.getX();
        double yx = desiredState.poseMeters.getY();
        double thetax = desiredRot.getRadians();

        double xv = desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getCos();
        double yv = desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getSin();

        // TODO: account for centripetal acceleration
        double xa = desiredState.accelerationMetersPerSecondSq * desiredState.poseMeters.getRotation().getCos();
        double ya = desiredState.accelerationMetersPerSecondSq * desiredState.poseMeters.getRotation().getSin();

        return new SwerveState(
                new State100(xx, xv, xa),
                new State100(yx, yv, ya),
                new State100(thetax, 0, 0));
    }

    /**
     * Transform timed pose into swerve state.
     * 
     * does not account for centripetal acceleration.
     */
    public static SwerveState fromTimedPose(TimedPose timedPose) {
        double xx = timedPose.state().getPose().getX();
        double yx = timedPose.state().getPose().getY();
        double thetax = timedPose.state().getHeading().getRadians();

        double velocityM_s = timedPose.velocityM_S();
        Optional<Rotation2d> course = timedPose.state().getCourse();
        Rotation2d motion_direction = course.isPresent() ? course.get() : GeometryUtil.kRotationZero;
        double xv = motion_direction.getCos() * velocityM_s;
        double yv = motion_direction.getSin() * velocityM_s;
        double thetav = timedPose.state().getHeadingRate() * velocityM_s;

        // TODO: account for centripetal acceleration
        double accelM_s_s = timedPose.acceleration();
        double xa = motion_direction.getCos() * accelM_s_s;
        double ya = motion_direction.getSin() * accelM_s_s;
        double thetaa = timedPose.state().getHeadingRate() * accelM_s_s;

        return new SwerveState(
                new State100(xx, xv, xa),
                new State100(yx, yv, ya),
                new State100(thetax, thetav, thetaa));
    }

    public String toString() {
        return "SwerveState(" + m_x + ", " + m_y + ", " + m_theta + ")";
    }

}