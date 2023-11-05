package org.team100.lib.spline;

import java.util.Optional;

import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

// 't' is the internal parameter, which usually isn't time.
public abstract class PoseSpline {
    public abstract Translation2d getPoint(double t);

    public abstract Rotation2d getHeading(double t);

    // Course i.e. motion direction. May be different from heading for holonomic motions.
    public abstract Optional<Rotation2d> getCourse(double t);

    // dtheta/dt (radians per t) - if you want radians per meter, use getDHeading / getVelocity.
    public abstract double getDHeading(double t);

    // inverse meters.
    // Curvature tells us how the motion direction is changing, whereas DHeading tells us how facing direction is changing.
    public abstract double getCurvature(double t);

    // dk/dt (inverse meters per t)
    public abstract double getDCurvature(double t);

    // ds/dt (meters per t)
    public abstract double getVelocity(double t);

    public Pose2d getPose2d(double t) {
        return new Pose2d(getPoint(t), getHeading(t));
    }

    public Pose2dWithMotion getPose2dWithMotion(double t) {
        var course = getCourse(t);
        double dx = course.isPresent() ? course.get().getCos() : 0.0;
        double dy = course.isPresent() ? course.get().getSin() : 0.0;
        double dtheta = course.isPresent() ? getDHeading(t) / getVelocity(t) : getDHeading(t);
        Twist2d motion_direction = new Twist2d(dx, dy, dtheta);
        return new Pose2dWithMotion(getPose2d(t), motion_direction, getCurvature(t), getDCurvature(t) / getVelocity(t));
    }
}