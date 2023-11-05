package org.team100.lib.geometry;


import java.text.DecimalFormat;
import java.util.Optional;

import org.team100.lib.util.MathUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/** This is from 254 2023 motion planning; the main reason to include it
 * is because it separates heading and course.
 */
public class Pose2dWithMotion {
    protected static final Pose2dWithMotion kIdentity = new Pose2dWithMotion();

    public static Pose2dWithMotion identity() {
        return kIdentity;
    }

    protected final Pose2d pose_;

    // Even though this Twist provides scalar values for dx, dy, and dtheta, Pose2dWithMotion is purely a spatial construct -
    // it has no sense of time. So rather than being in units of distance-per-time (or radians-per-time), the denominator here is
    // actually distance as well. Thus, it isn't really meaningful to look at dx or dy directly - what is meaningful is:
    //  a) whether or not they are both zero (in which case this is a stationary or turn-in-place motion)
    //  b) the angle formed by them, which is the direction of translation (in the same coordinate frame as pose_).
    // Additionally, this means dtheta is in radians-per-distance if there is translation, or radians-per-radian otherwise.
    protected final Twist2d motion_direction_;

    protected final double curvature_;
    protected final double dcurvature_ds_;

    public Pose2dWithMotion() {
        pose_ = new Pose2d();
        motion_direction_ = new Twist2d(0.0, 0.0, 0.0);
        curvature_ = 0.0;
        dcurvature_ds_ = 0.0;
    }

    public Pose2dWithMotion(final Pose2d pose, double curvature) {
        pose_ = pose;
        motion_direction_ = new Twist2d(0.0, 0.0, 0.0);
        curvature_  = curvature;
        dcurvature_ds_ = 0.0;
    }

    public Pose2dWithMotion(final Pose2d pose, double curvature, double dcurvature_ds) {
        pose_ = pose;
        motion_direction_ = new Twist2d(0.0, 0.0, 0.0);
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }


    public Pose2dWithMotion(final Pose2d pose, final Twist2d motion_direction, double curvature, double dcurvature_ds) {
        pose_ = pose;
        motion_direction_ = motion_direction;
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    public Pose2dWithMotion(final Translation2d translation, final Rotation2d rotation, double curvature) {
        pose_ = new Pose2d(translation, rotation);
        motion_direction_ = new Twist2d(0.0, 0.0, 0.0);
        curvature_  = curvature;
        dcurvature_ds_ = 0.0;
    }

    public Pose2dWithMotion(final Translation2d translation, final Rotation2d rotation, double curvature, double dcurvature_ds) {
        pose_ = new Pose2d(translation, rotation);
        motion_direction_ = new Twist2d(0.0, 0.0, 0.0);
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    public Pose2dWithMotion(final Translation2d translation, final Rotation2d rotation, final Twist2d motion_direction, double curvature, double dcurvature_ds) {
        pose_ = new Pose2d(translation, rotation);
        motion_direction_ = motion_direction;
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    public final Pose2d getPose() {
        return pose_;
    }

    public Pose2dWithMotion transformBy(Pose2d transform) {
        return new Pose2dWithMotion(GeometryUtil.transformBy(getPose(), transform), motion_direction_, getCurvature(), getDCurvatureDs());
    }

    public Pose2dWithMotion mirror() {
        return new Pose2dWithMotion(GeometryUtil.mirror(getPose()), GeometryUtil.mirror(motion_direction_), -getCurvature(), -getDCurvatureDs());
    }

    public double getCurvature() {
        return curvature_;
    }

    public double getDCurvatureDs() {
        return dcurvature_ds_;
    }

    public final Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public final Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public Pose2dWithMotion interpolate(final Pose2dWithMotion other, double x) {
        return new Pose2dWithMotion(getPose().interpolate(other.getPose(), x),
                GeometryUtil.interpolate(motion_direction_, other.motion_direction_, x),
                MathUtil.interpolate(getCurvature(), other.getCurvature(), x),
                MathUtil.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    public double distance(final Pose2dWithMotion other) {
        return GeometryUtil.distance(getPose(), other.getPose());
    }

    public boolean equals(final Object other) {
        if (!(other instanceof Pose2dWithMotion)) {
            return false;
        }

        Pose2dWithMotion p2dwc = (Pose2dWithMotion) other;
        return getPose().equals(p2dwc.getPose()) &&
            motion_direction_.equals(p2dwc.motion_direction_) &&
            MathUtil.epsilonEquals(getCurvature(), p2dwc.getCurvature()) &&
            MathUtil.epsilonEquals(getDCurvatureDs(), p2dwc.getDCurvatureDs());
    }

    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toString() + ", twist: " + motion_direction_ + ", curvature: " + fmt.format(getCurvature()) + ", dcurvature_ds: " + fmt.format(getDCurvatureDs());
    }

    public Pose2dWithMotion rotateBy(Rotation2d other) {
        // motion direction is always relative to pose, so it gets rotated "for free".
        return new Pose2dWithMotion(getPose().rotateBy(other), getCurvature(), getDCurvatureDs());
    }

    public Pose2dWithMotion add(Pose2dWithMotion other) {
        return this.transformBy(other.getPose());   // todo make work
    }

    public Optional<Rotation2d> getCourseLocalFrame() {
        var course = getCourse();
        if (course.isEmpty()) { return course; }
        return Optional.of(getRotation().unaryMinus().rotateBy(course.get()));
    }

    public Optional<Rotation2d> getCourse() {
        return GeometryUtil.getCourse(motion_direction_);
    }

    public double getHeadingRate() {
        return motion_direction_.dtheta;
    }
}