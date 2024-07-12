package org.team100.lib.spline;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class SplineGenerator {
    /**
     * Converts a spline into a list of Twist2d's.
     *
     * @param s  the spline to parametrize
     * @param t0 starting percentage of spline to parametrize
     * @param t1 ending percentage of spline to parametrize
     * @return list of Pose2dWithCurvature that approximates the original spline
     */
    public static List<Pose2dWithMotion> parameterizeSpline(
            HolonomicSpline s,
            double maxDx,
            double maxDy,
            double maxDTheta,
            double t0,
            double t1) {
        List<Pose2dWithMotion> rv = new ArrayList<>();
        rv.add(s.getPose2dWithMotion(0.0));
        double dt = (t1 - t0);
        for (double t = 0; t < t1; t += dt) {
            getSegmentArc(s, rv, t, t + dt, maxDx, maxDy, maxDTheta);
        }
        return rv;
    }

    public static List<Pose2dWithMotion> parameterizeSplines(
            List<? extends HolonomicSpline> splines,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        List<Pose2dWithMotion> rv = new ArrayList<>();
        if (splines.isEmpty())
            return rv;
        rv.add(splines.get(0).getPose2dWithMotion(0.0));
        for (int i = 0; i < splines.size(); i++) {
            HolonomicSpline s = splines.get(i);
            List<Pose2dWithMotion> samples = parameterizeSpline(s, maxDx, maxDy, maxDTheta, 0.0, 1.0);
            samples.remove(0);
            rv.addAll(samples);
        }
        return rv;
    }

    private static void getSegmentArc(
            HolonomicSpline s,
            List<Pose2dWithMotion> rv,
            double t0,
            double t1,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        Pose2d p0 = s.getPose2d(t0);
        Pose2d phalf = s.getPose2d(t0 + (t1 - t0) * .5);
        Pose2d p1 = s.getPose2d(t1);
        Twist2d twist_full = GeometryUtil.kPoseZero.log(GeometryUtil.transformBy(GeometryUtil.inverse(p0), p1));
        Pose2d phalf_predicted = GeometryUtil.transformBy(p0,
                GeometryUtil.kPoseZero.exp(GeometryUtil.scale(twist_full, 0.5)));
        Pose2d error = GeometryUtil.transformBy(GeometryUtil.inverse(phalf), phalf_predicted);
        
        if (GeometryUtil.norm(twist_full) < 1e-6) {
            // the Rotation2d below will be garbage in this case so give up.
            return;
        }
        Rotation2d course_predicted = (new Rotation2d(twist_full.dx, twist_full.dy))
                .rotateBy(phalf_predicted.getRotation());

        Rotation2d course_half = s.getCourse(t0 + (t1 - t0) * .5).orElse(course_predicted);
        double course_error = course_predicted.unaryMinus().rotateBy(course_half).getRadians();
        if (Math.abs(error.getTranslation().getY()) > maxDy ||
                Math.abs(error.getTranslation().getX()) > maxDx ||
                Math.abs(error.getRotation().getRadians()) > maxDTheta ||
                Math.abs(course_error) > maxDTheta) {
            getSegmentArc(s, rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
            getSegmentArc(s, rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
        } else {
            rv.add(s.getPose2dWithMotion(t1));
        }
    }

    private SplineGenerator() {
    }
}
