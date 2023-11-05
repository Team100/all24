package org.team100.lib.geometry;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.PoseWithCurvature;

public class GeometryUtil {

    public static final Rotation2d kRotationIdentity = new Rotation2d();
    public static final Rotation2d kPi = new Rotation2d(Math.PI);
    public static final Pose2d kPose2dIdentity = new Pose2d();
    public static final Translation2d kTranslation2dIdentity = new Translation2d();
    public static final PoseWithCurvature kPose2dWithCurvatureIdentity = new PoseWithCurvature();
    public static final Twist2d kTwist2dIdentity = new Twist2d(0.0, 0.0, 0.0);

    private GeometryUtil() {
    }

    public static Twist2d scale(Twist2d twist, double scale) {
        return new Twist2d(twist.dx * scale, twist.dy * scale, twist.dtheta * scale);
    }

    public static Pose2d transformBy(Pose2d a, Pose2d b) {
        return a.transformBy(new Transform2d(b.getTranslation(), b.getRotation()));
    }

    public static Pose2d inverse(Pose2d a) {
        Rotation2d rotation_inverted = a.getRotation().unaryMinus();
        return new Pose2d(a.getTranslation().unaryMinus().rotateBy(rotation_inverted), rotation_inverted);
    }

    public static Twist2d slog(final Pose2d transform) {
        return new Pose2d().log(transform);
    }

    public static Pose2d sexp(final Twist2d delta) {
        return new Pose2d().exp(delta);
    }

    public static Pose2d fromRotation(final Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    public static Pose2d fromTranslation(final Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }

    public static Rotation2d fromRadians(double angle_radians) {
        return new Rotation2d(angle_radians);
    }

    public static Rotation2d fromDegrees(double angle_degrees) {
        return fromRadians(Math.toRadians(angle_degrees));
    }

    public static double WrapRadians(double radians) {
        final double k2Pi = 2.0 * Math.PI;
        radians = radians % k2Pi;
        radians = (radians + k2Pi) % k2Pi;
        if (radians > Math.PI)
            radians -= k2Pi;
        return radians;
    }

    public static boolean isColinear(Pose2d a, final Pose2d other) {
        if (!GeometryUtil.isParallel(a.getRotation(), other.getRotation()))
            return false;
        final Twist2d twist = slog(transformBy(inverse(a), other));
        return (Math.abs(twist.dy - 0.0) <= 1e-12 && Math.abs(twist.dtheta - 0.0) <= 1e-12);
    }

    // note parallel also means antiparallel.
    public static boolean isParallel(Rotation2d a, Rotation2d b) {
        return Math.abs(a.getRadians() - b.getRadians()) <= 1e-12
                || Math.abs(a.getRadians() - WrapRadians(b.getRadians() + Math.PI)) <= 1e-12;
    }

    public static double norm(Twist2d a) {
        // Common case of dy == 0
        if (a.dy == 0.0)
            return Math.abs(a.dx);
        return Math.hypot(a.dx, a.dy);
    }

    public static Rotation2d flip(Rotation2d a) {
        return new Rotation2d(a.getRadians() + Math.PI);
    }

    public static double distance(Rotation2d a, final Rotation2d other) {
        return a.unaryMinus().rotateBy(other).getRadians();
    }

    public static Rotation2d interpolate2(Rotation2d a, final Rotation2d b, double x) {
        if (x <= 0.0) {
            return a;
        } else if (x >= 1.0) {
            return b;
        }
        double angle_diff = a.unaryMinus().rotateBy(b).getRadians();
        return a.rotateBy(Rotation2d.fromRadians(angle_diff * x));
    }

    public static double distance(PoseWithCurvature a, PoseWithCurvature b) {
        // this is not used
        return norm(slog(transformBy(inverse(a.poseMeters), b.poseMeters)));
    }

    public static double distance(Pose2d a, Pose2d b) {
        return norm(slog(transformBy(inverse(a), b)));
    }

    public static double distance(Translation2d a, Translation2d b) {
        return inverse(a).plus(b).getNorm();
    }

    public static Translation2d inverse(Translation2d a) {
        return new Translation2d(-a.getX(), -a.getY());
    }

    public static PoseWithCurvature interpolate2(PoseWithCurvature a, final PoseWithCurvature other, double x) {
        Pose2d interpolatedPose = a.poseMeters.interpolate(other.poseMeters, x);
        double interpolatedCurvature = MathUtil.interpolate(a.curvatureRadPerMeter, other.curvatureRadPerMeter, x);
        return new PoseWithCurvature(interpolatedPose, interpolatedCurvature);
    }

    public static Twist2d interpolate(Twist2d a, Twist2d b, double x) {
        return new Twist2d(MathUtil.interpolate(a.dx, b.dx, x),
                MathUtil.interpolate(a.dy, b.dy, x),
                MathUtil.interpolate(a.dtheta, b.dtheta, x));
    }

    public static boolean poseWithCurvatureEquals(PoseWithCurvature a, PoseWithCurvature b) {
        boolean poseEqual = a.poseMeters.equals(b.poseMeters);
        if (!poseEqual) {
            System.out.println("pose not equal");
            return false;
        }
        boolean curvatureEqual = Math.abs(a.curvatureRadPerMeter - b.curvatureRadPerMeter) <= 1e-12;
        if (!curvatureEqual) {
            System.out.println("curvature not equal");
            return false;
        }
        return true;
    }

    public static Optional<Rotation2d> getCourse(Twist2d t) {
        if (norm(t) > 1e-12) {
            return Optional.of(new Rotation2d(t.dx, t.dy));
        } else {
            return Optional.empty();
        }
    }

    public static Twist2d mirror(Twist2d t) {
        return new Twist2d(t.dx, -t.dy, -t.dtheta);
    }

    public static Pose2d mirror(Pose2d p) {
        return new Pose2d(new Translation2d(p.getTranslation().getX(), -p.getTranslation().getY()), p.getRotation().unaryMinus());
    }

    public static Twist2d toTwist2d(ChassisSpeeds x) {
        return new Twist2d(x.vxMetersPerSecond, x.vyMetersPerSecond, x.omegaRadiansPerSecond);
    }

}
