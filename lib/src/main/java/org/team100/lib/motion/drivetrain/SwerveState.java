package org.team100.lib.motion.drivetrain;

import org.team100.lib.controller.State100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
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

    /**
     * SwerveState with the specified field-relative pose and field-relative
     * velocity, and zero accelerations.
     */
    public SwerveState(Pose2d p, Twist2d t) {
        this(
                new State100(p.getX(), t.dx, 0),
                new State100(p.getY(), t.dy, 0),
                new State100(p.getRotation().getRadians(), t.dtheta, 0));
    }

    public Pose2d pose() {
        return new Pose2d(m_x.x(), m_y.x(), new Rotation2d(m_theta.x()));
    }

    /** Translation of the pose */
    public Translation2d translation() {
        return new Translation2d(m_x.x(), m_y.x());
    }

    public Twist2d twist() {
        return new Twist2d(m_x.v(), m_y.v(), m_theta.v());
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
     * TODO: stop using wpi trajectory state.
     */
    public static SwerveState fromState(State desiredState, Rotation2d desiredRot) {
        double xx = desiredState.poseMeters.getX();
        double yx = desiredState.poseMeters.getY();
        double thetax = desiredRot.getRadians();

        double xv = desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getCos();
        double yv = desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getSin();

        double xa = desiredState.accelerationMetersPerSecondSq * desiredState.poseMeters.getRotation().getCos();
        double ya = desiredState.accelerationMetersPerSecondSq * desiredState.poseMeters.getRotation().getSin();

        // TODO: thetav, thetaa.

        return new SwerveState(
                new State100(xx, xv, xa),
                new State100(yx, yv, ya),
                new State100(thetax, 0, 0));

    }

    public String toString() {
        return "SwerveState(" + m_x + ", " + m_y + ", " + m_theta + ")";
    }

}