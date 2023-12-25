package org.team100.lib.spline;

import java.util.Optional;

import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Represents a sequence of poses, parameterized by "p," which is usually not
 * time.
 */
public abstract class PoseSpline {
    /**
     * Cartesian coordinate in meters at p.
     */
    protected abstract Translation2d getPoint(double p);

    /**
     * Heading is the direction the robot is facing, regardless of the direction of
     * motion (course).
     */
    protected abstract Rotation2d getHeading(double p);

    /**
     * Course is the direction of motion, regardless of the direction the robot is
     * facing (heading). It's optional to account for the motionless case.
     */
    public abstract Optional<Rotation2d> getCourse(double t);

    /**
     * DHeading is the change in heading per parameter, p.
     * dtheta/dp (radians per p).
     * If you want radians per meter, use getDHeadingDs.
     */
    protected abstract double getDHeading(double p);

    /**
     * Change in heading per distance traveled, i.e. spatial change in heading.
     * dtheta/ds (radians/meter).
     */
    private double getDHeadingDs(double p) {
        return getDHeading(p) / getVelocity(p);
    }

    /**
     * Curvature is the change in motion direction per distance traveled.
     * rad/m.
     * Note the denominator is distance in this case, not the parameter, p.
     */
    public abstract double getCurvature(double p);

    /**
     * DCurvature is the change in curvature per change in p.
     * dk/dp (rad/m per p)
     * If you want change in curvature per meter, use getDCurvatureDs.
     */
    protected abstract double getDCurvature(double p);

    /**
     * DCurvatureDs is the change in curvature per distance traveled, i.e. the
     * "spatial change in curvature"
     * 
     * dk/dp / ds/dp = dk/ds
     * rad/mp / m/p = rad/m^2
     */
    private double getDCurvatureDs(double p) {
        return getDCurvature(p) / getVelocity(p);
    }

    /**
     * Velocity is the change in position per parameter, p: ds/dp (meters per p).
     * Since p is not time, it is not "velocity" in the usual sense.
     */
    protected abstract double getVelocity(double p);


    Pose2d getPose2d(double p) {
        return new Pose2d(getPoint(p), getHeading(p));
    }

    public Pose2dWithMotion getPose2dWithMotion(double p) {
        Optional<Rotation2d> course = getCourse(p);
        double dx = course.isPresent() ? course.get().getCos() : 0.0;
        double dy = course.isPresent() ? course.get().getSin() : 0.0;
        double dtheta = course.isPresent() ? getDHeadingDs(p) : getDHeading(p);

        return new Pose2dWithMotion(
                getPose2d(p),
                new Twist2d(dx, dy, dtheta),
                getCurvature(p),
                getDCurvatureDs(p));
    }
}