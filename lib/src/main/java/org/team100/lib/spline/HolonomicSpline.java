package org.team100.lib.spline;

import java.util.List;
import java.util.Optional;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Holonomic spline.
 * 
 * Internally this is three one-dimensional splines (x, y, heading), with
 * respect to a parameter [0,1].
 * 
 * If you don't care about rotation, just pass zero.
 * 
 * Note that some nonholonomic spline consumers assume that dx carries all the
 * motion; that's not true here.
 */
public class HolonomicSpline {
    /** spline control points need to be not too close to a u-turn. */
    private static final double MIN_ANGLE = 2 * Math.PI / 3;
    private static final double kEpsilon = 1e-5;
    private static final double kStepSize = 1.0;
    private static final double kMinDelta = 0.001;
    private static final int kSamples = 100;
    private static final int kMaxIterations = 100;

    private final Spline1d x;
    private final Spline1d y;
    private final Spline1d theta;
    private final Rotation2d r0;

    /**
     * Note: the p0 direction needs to be within 90 degrees of p1-p0, and vice
     * versa, to prevent "bad" trajectories, where the robot ends up accelerating
     * away from the target. If you really want a u-turn, add a stopping point.
     * 
     * @param p0 The starting point and direction of the spline
     * @param p1 The ending point and direction of the spline
     * @param r0 The starting heading
     * @param r1 The ending heading
     */
    public HolonomicSpline(Pose2d p0, Pose2d p1, Rotation2d r0, Rotation2d r1) {
        checkBounds(p0, p1);
        // the 1.2 here is a magic number that makes the spline look nice.
        double scale = 1.2 * GeometryUtil.distance(p0.getTranslation(), p1.getTranslation());
        double x0 = p0.getTranslation().getX();
        double x1 = p1.getTranslation().getX();
        double dx0 = p0.getRotation().getCos() * scale;
        double dx1 = p1.getRotation().getCos() * scale;
        double ddx0 = 0;
        double ddx1 = 0;
        double y0 = p0.getTranslation().getY();
        double y1 = p1.getTranslation().getY();
        double dy0 = p0.getRotation().getSin() * scale;
        double dy1 = p1.getRotation().getSin() * scale;
        double ddy0 = 0;
        double ddy1 = 0;

        this.x = Spline1d.newSpline1d(x0, x1, dx0, dx1, ddx0, ddx1);
        this.y = Spline1d.newSpline1d(y0, y1, dy0, dy1, ddy0, ddy1);
        this.r0 = r0;
        double delta = r0.unaryMinus().rotateBy(r1).getRadians();
        theta = Spline1d.newSpline1d(0.0, delta, 0, 0, 0, 0);
    }

    static void checkBounds(Pose2d p0, Pose2d p1) {
        Translation2d toTarget = p1.getTranslation().minus(p0.getTranslation());
        Rotation2d angle = toTarget.getAngle();
        double p0Angle = Math.abs(MathUtil.angleModulus(p0.getRotation().minus(angle).getRadians()));
        if (p0Angle > MIN_ANGLE)
            throw new IllegalArgumentException(
                    String.format("p0 direction, %.3f is too far from course to p1, %.3f -- P0 %s P1 %s",
                            p0.getRotation().getRadians(), angle.getRadians(), p0, p1));
        double p1Angle = Math.abs(MathUtil.angleModulus(p1.getRotation().minus(angle).getRadians()));
        if (p1Angle > MIN_ANGLE)
            throw new IllegalArgumentException(
                    String.format("p1 direction, %.3f is too far from course from p0, %.3f -- P0 %s P1 %s",
                            p1.getRotation().getRadians(), angle.getRadians(), p0, p1));
    }

    private HolonomicSpline(
            Spline1d x,
            Spline1d y,
            Spline1d theta,
            Rotation2d r0) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.r0 = r0;
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

    /**
     * Course is the direction of motion, regardless of the direction the robot is
     * facing (heading). It's optional to account for the motionless case.
     *
     * Course is the same for holonomic and nonholonomic splines.
     */
    public Optional<Rotation2d> getCourse(double t) {
        double dx = dx(t);
        double dy = dy(t);
        if (Math100.epsilonEquals(dx, 0.0) && Math100.epsilonEquals(dy, 0.0)) {
            // rotation below would be garbage so give up
            return Optional.empty();
        }
        return Optional.of(new Rotation2d(dx, dy));
    }

    /**
     * Finds the optimal second derivative values for a set of splines to reduce the
     * sum of the change in curvature squared over the path
     *
     * @param splines the list of splines to optimize
     * @return the final sumDCurvature2
     */
    public static double optimizeSpline(List<HolonomicSpline> splines) {
        int count = 0;
        double prev = sumDCurvature2(splines);
        while (count < kMaxIterations) {
            runOptimizationIteration(splines);
            double current = sumDCurvature2(splines);
            if (prev - current < kMinDelta)
                return current;
            prev = current;
            count++;
        }
        Util.warn("Spline optimization failed");
        return prev;
    }

    Pose2d getPose2d(double p) {
        return new Pose2d(getPoint(p), getHeading(p));
    }

    ////////////////////////////////////////////////////////////////////////

    protected Rotation2d getHeading(double t) {
        return r0.rotateBy(Rotation2d.fromRadians(theta.getPosition(t)));
    }

    protected double getDHeading(double t) {
        return theta.getVelocity(t);
    }

    /**
     * Return a new spline that is a copy of this one, but with substitute
     * second derivatives.
     */
    private HolonomicSpline adjustSecondDerivatives(
            double ddx0_sub, double ddx1_sub,
            double ddy0_sub, double ddy1_sub) {
        return new HolonomicSpline(
                x.addCoefs(Spline1d.newSpline1d(0, 0, 0, 0, ddx0_sub, ddx1_sub)),
                y.addCoefs(Spline1d.newSpline1d(0, 0, 0, 0, ddy0_sub, ddy1_sub)),
                theta,
                r0);
    }

    /**
     * Change in heading per distance traveled, i.e. spatial change in heading.
     * dtheta/ds (radians/meter).
     */
    private double getDHeadingDs(double p) {
        return getDHeading(p) / getVelocity(p);
    }

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

    /** Returns pose in the nonholonomic sense, where the rotation is the course */
    private Optional<Pose2d> getStartPose() {
        double dx = dx(0);
        double dy = dy(0);
        if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) {
            // rotation below would be garbage, so give up.
            return Optional.empty();
        }
        return Optional.of(new Pose2d(
                getPoint(0),
                new Rotation2d(dx, dy)));
    }

    /** Returns pose in the nonholonomic sense, where the rotation is the course */
    private Optional<Pose2d> getEndPose() {
        double dx = dx(1);
        double dy = dy(1);
        if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) {
            // rotation below would be garbage, so give up.
            return Optional.empty();
        }
        return Optional.of(new Pose2d(
                getPoint(1),
                new Rotation2d(dx, dy)));
    }

    /**
     * Cartesian coordinate in meters at p.
     * 
     * @param t ranges from 0 to 1
     * @return the point on the spline for that t value
     */
    protected Translation2d getPoint(double t) {
        return new Translation2d(x.getPosition(t), y.getPosition(t));
    }

    double dx(double t) {
        return x.getVelocity(t);
    }

    private double dy(double t) {
        return y.getVelocity(t);
    }

    double ddx(double t) {
        return x.getAcceleration(t);
    }

    private double ddy(double t) {
        return y.getAcceleration(t);
    }

    private double dddx(double t) {
        return x.getJerk(t);
    }

    private double dddy(double t) {
        return y.getJerk(t);
    }

    /**
     * Velocity is the change in position per parameter, p: ds/dp (meters per p).
     * Since p is not time, it is not "velocity" in the usual sense.
     */
    protected double getVelocity(double t) {
        return Math.hypot(dx(t), dy(t));
    }

    /**
     * Curvature is the change in motion direction per distance traveled.
     * rad/m.
     * Note the denominator is distance in this case, not the parameter, p.
     * but the argument to this function *is* the parameter, p. :-)
     */
    protected double getCurvature(double t) {
        return (dx(t) * ddy(t) - ddx(t) * dy(t))
                / ((dx(t) * dx(t) + dy(t) * dy(t)) * Math.sqrt((dx(t) * dx(t) + dy(t) * dy(t))));
    }

    /**
     * DCurvature is the change in curvature per change in p.
     * dk/dp (rad/m per p)
     * If you want change in curvature per meter, use getDCurvatureDs.
     */
    protected double getDCurvature(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2
                - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num / (dx2dy2 * dx2dy2 * Math.sqrt(dx2dy2));
    }

    private double dCurvature2(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        if (dx2dy2 == 0)
            throw new IllegalArgumentException();
        double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2
                - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
    }

    /** integrate curvature over the length of the spline. */
    double maxCurvature() {
        double dt = 1.0 / kSamples;
        double maxC = 0;
        for (double t = 0; t < 1.0; t += dt) {
            maxC = Math.max(maxC, getCurvature(t));
        }
        return maxC;
    }

    /** integrate curvature over the length of the spline. */
    double sumCurvature() {
        double dt = 1.0 / kSamples;
        double sum = 0;
        for (double t = 0; t < 1.0; t += dt) {
            sum += (dt * getCurvature(t));
        }
        return sum;
    }

    /**
     * @return integral of dCurvature^2 over the length of the spline
     */
    double sumDCurvature2() {
        double dt = 1.0 / kSamples;
        double sum = 0;
        for (double t = 0; t < 1.0; t += dt) {
            sum += (dt * dCurvature2(t));
        }
        return sum;
    }

    /**
     * @return integral of dCurvature^2 over the length of multiple splines
     */
    static double sumDCurvature2(List<HolonomicSpline> splines) {
        double sum = 0;
        for (HolonomicSpline s : splines) {
            sum += s.sumDCurvature2();
        }
        if (Double.isNaN(sum))
            throw new IllegalArgumentException();
        return sum;
    }

    /**
     * Makes optimization code a little more readable
     */
    private static class ControlPoint {
        private double ddx;
        private double ddy;
    }

    /**
     * Runs a single optimization iteration
     */
    private static void runOptimizationIteration(List<HolonomicSpline> splines) {
        // can't optimize anything with less than 2 splines
        if (splines.size() <= 1) {
            // Util.warn("runOptimizationIteration: nothing to optimize");
            return;
        }

        ControlPoint[] controlPoints = new ControlPoint[splines.size() - 1];

        double magnitude = getControlPoints(splines, controlPoints);

        magnitude = Math.sqrt(magnitude);
        if (Double.isNaN(magnitude))
            throw new IllegalArgumentException();

        // minimize along the direction of the gradient
        // first calculate 3 points along the direction of the gradient

        // middle point is at the current location
        Translation2d p2 = new Translation2d(0, sumDCurvature2(splines));

        // first point is offset from the middle location by -stepSize
        for (int i = 0; i < splines.size() - 1; ++i) {
            backwards(splines, controlPoints, magnitude, i);
        }

        // last point is offset from the middle location by +stepSize
        Translation2d p1 = new Translation2d(-kStepSize, sumDCurvature2(splines));
        for (int i = 0; i < splines.size() - 1; ++i) {
            forwards(splines, controlPoints, i);
        }

        Translation2d p3 = new Translation2d(kStepSize, sumDCurvature2(splines));
        // approximate step size to minimize sumDCurvature2 along the gradient
        double stepSize = fitParabola(p1, p2, p3);

        for (int i = 0; i < splines.size() - 1; ++i) {
            finish(splines, controlPoints, stepSize, i);
        }
    }

    private static void finish(List<HolonomicSpline> splines, ControlPoint[] controlPoints, double stepSize, int i) {
        Optional<Pose2d> startPose = splines.get(i).getStartPose();
        Optional<Pose2d> startPose2 = splines.get(i + 1).getStartPose();
        Optional<Pose2d> endPose = splines.get(i).getEndPose();
        Optional<Pose2d> endPose2 = splines.get(i + 1).getEndPose();
        if (startPose.isEmpty() || startPose2.isEmpty() || endPose.isEmpty() || endPose2.isEmpty()) {
            throw new IllegalArgumentException();
        }
        if (GeometryUtil.isColinear(startPose.get(), startPose2.get())
                || GeometryUtil.isColinear(endPose.get(), endPose2.get())) {
            return;
        }

        // why would this happen?
        if (controlPoints[i] == null)
            return;

        // move by the step size calculated by the parabola fit (+1 to offset for the
        // final transformation to find p3)
        controlPoints[i].ddx *= 1 + stepSize / kStepSize;
        controlPoints[i].ddy *= 1 + stepSize / kStepSize;

        splines.set(i, splines.get(i).adjustSecondDerivatives(0, controlPoints[i].ddx, 0, controlPoints[i].ddy));
        splines.set(i + 1,
                splines.get(i + 1).adjustSecondDerivatives(controlPoints[i].ddx, 0, controlPoints[i].ddy, 0));
    }

    private static void forwards(List<HolonomicSpline> splines, ControlPoint[] controlPoints, int i) {
        Optional<Pose2d> startPose = splines.get(i).getStartPose();
        Optional<Pose2d> startPose2 = splines.get(i + 1).getStartPose();
        Optional<Pose2d> endPose = splines.get(i).getEndPose();
        Optional<Pose2d> endPose2 = splines.get(i + 1).getEndPose();
        if (startPose.isEmpty() || startPose2.isEmpty() || endPose.isEmpty() || endPose2.isEmpty()) {
            throw new IllegalArgumentException();
        }
        if (GeometryUtil.isColinear(startPose.get(), startPose2.get())
                || GeometryUtil.isColinear(endPose.get(), endPose2.get())) {
            return;
        }

        // why would this happen?
        if (controlPoints[i] == null)
            return;

        // move along the gradient by 2 times the step size amount (to return to
        // original location and move by 1 step)
        splines.set(i,
                splines.get(i).adjustSecondDerivatives(0, 2 * controlPoints[i].ddx, 0, 2 * controlPoints[i].ddy));
        splines.set(i + 1, splines.get(i + 1).adjustSecondDerivatives(2 * controlPoints[i].ddx, 0,
                2 * controlPoints[i].ddy, 0));
    }

    private static void backwards(
            List<HolonomicSpline> splines,
            ControlPoint[] controlPoints,
            double magnitude,
            int i) {
        Optional<Pose2d> startPose = splines.get(i).getStartPose();
        Optional<Pose2d> startPose2 = splines.get(i + 1).getStartPose();
        Optional<Pose2d> endPose = splines.get(i).getEndPose();
        Optional<Pose2d> endPose2 = splines.get(i + 1).getEndPose();
        if (startPose.isEmpty() || startPose2.isEmpty() || endPose.isEmpty() || endPose2.isEmpty()) {
            throw new IllegalArgumentException();
        }
        if (GeometryUtil.isColinear(startPose.get(), startPose2.get())
                || GeometryUtil.isColinear(endPose.get(), endPose2.get())) {
            return;
        }

        // why would this happen?
        if (controlPoints[i] == null)
            return;

        // normalize to step size
        controlPoints[i].ddx *= kStepSize / magnitude;
        controlPoints[i].ddy *= kStepSize / magnitude;

        // move opposite the gradient by step size amount
        splines.set(i, splines.get(i).adjustSecondDerivatives(0, -controlPoints[i].ddx, 0, -controlPoints[i].ddy));
        splines.set(i + 1,
                splines.get(i + 1).adjustSecondDerivatives(-controlPoints[i].ddx, 0, -controlPoints[i].ddy, 0));
    }

    /**
     * Extract the control points from the list of splines.
     * 
     * @param splines       input splines
     * @param controlPoints output control points
     * @return sum of ddx^2+ddy^2
     */
    private static double getControlPoints(List<HolonomicSpline> splines, ControlPoint[] controlPoints) {
        double magnitude = 0;
        for (int i = 0; i < splines.size() - 1; ++i) {
            // don't try to optimize colinear points
            Optional<Pose2d> startPose = splines.get(i).getStartPose();
            Optional<Pose2d> startPose2 = splines.get(i + 1).getStartPose();
            Optional<Pose2d> endPose = splines.get(i).getEndPose();
            Optional<Pose2d> endPose2 = splines.get(i + 1).getEndPose();
            if (startPose.isEmpty() || startPose2.isEmpty() || endPose.isEmpty() || endPose2.isEmpty()) {
                throw new IllegalArgumentException();
            }
            if (GeometryUtil.isColinear(startPose.get(), startPose2.get())
                    || GeometryUtil.isColinear(endPose.get(), endPose2.get())) {
                continue;
            }
            double original = sumDCurvature2(splines);

            // holds the gradient at a control point
            controlPoints[i] = new ControlPoint();

            // calculate partial derivatives of sumDCurvature2
            splines.set(i, splines.get(i).adjustSecondDerivatives(0, kEpsilon, 0, 0));
            splines.set(i + 1, splines.get(i + 1).adjustSecondDerivatives(kEpsilon, 0, 0, 0));
            controlPoints[i].ddx = (sumDCurvature2(splines) - original) / kEpsilon;

            splines.set(i, splines.get(i).adjustSecondDerivatives(0, 0, 0, kEpsilon));
            splines.set(i + 1, splines.get(i + 1).adjustSecondDerivatives(0, 0, kEpsilon, 0));
            controlPoints[i].ddy = (sumDCurvature2(splines) - original) / kEpsilon;

            splines.set(i, splines.get(i));
            splines.set(i + 1, splines.get(i + 1));
            magnitude += controlPoints[i].ddx * controlPoints[i].ddx + controlPoints[i].ddy * controlPoints[i].ddy;
        }
        return magnitude;
    }

    /**
     * fits a parabola to 3 points
     *
     * @return the x coordinate of the vertex of the parabola
     */
    private static double fitParabola(Translation2d p1, Translation2d p2, Translation2d p3) {
        double A = (p3.getX() * (p2.getY() - p1.getY()) + p2.getX() * (p1.getY() - p3.getY())
                + p1.getX() * (p3.getY() - p2.getY()));
        double B = (p3.getX() * p3.getX() * (p1.getY() - p2.getY()) + p2.getX() * p2.getX() * (p3.getY() - p1.getY())
                + p1.getX() * p1.getX() *
                        (p2.getY() - p3.getY()));
        return -B / (2 * A);
    }

}