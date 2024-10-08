package org.team100.lib.motion.drivetrain.kinodynamics;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Just like ChassisSpeeds, but field-relative, to avoid mixing them up.
 */
public record FieldRelativeVelocity(double x, double y, double theta) {

    public static FieldRelativeVelocity zero() {
        return new FieldRelativeVelocity(0, 0, 0);
    }

    public static FieldRelativeVelocity velocity(Pose2d start, Pose2d end, double dt) {
        FieldRelativeDelta d = FieldRelativeDelta.delta(start, end);
        return new FieldRelativeVelocity(d.getX(), d.getY(), d.getRotation().getRadians()).div(dt);
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public FieldRelativeVelocity normalize() {
        double norm = norm();
        if (norm < 1e-6)
            return zero();
        return new FieldRelativeVelocity(x, y, theta).times(1.0 / norm);
    }

    public Optional<Rotation2d> angle() {
        if (Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6)
            return Optional.empty();
        return Optional.of(new Rotation2d(x, y));
    }

    public FieldRelativeVelocity plus(FieldRelativeVelocity other) {
        return new FieldRelativeVelocity(x + other.x, y + other.y, theta + other.theta);
    }

    /** The return type here isn't really right. */
    public FieldRelativeVelocity minus(FieldRelativeVelocity other) {
        return new FieldRelativeVelocity(x - other.x, y - other.y, theta - other.theta);
    }

    public FieldRelativeAcceleration accel(FieldRelativeVelocity previous, double dt) {
        FieldRelativeVelocity v = minus(previous).div(dt);
        return new FieldRelativeAcceleration(v.x(), v.y(), v.theta());
    }

    public FieldRelativeVelocity times(double scalar) {
        return new FieldRelativeVelocity(x * scalar, y * scalar, theta * scalar);
    }

    public FieldRelativeVelocity div(double scalar) {
        return new FieldRelativeVelocity(x / scalar, y / scalar, theta / scalar);
    }

    public FieldRelativeVelocity times(double cartesian, double angular) {
        return new FieldRelativeVelocity(x * cartesian, y * cartesian, theta * angular);
    }

    /** Dot product of translational part. */
    public double dot(FieldRelativeVelocity other) {
        return x * other.x + y * other.y;
    }

    public FieldRelativeVelocity clamp(double maxVelocity, double maxOmega) {
        double norm = Math.hypot(x, y);
        double ratio = 1.0;
        if (norm > 1e-3 && norm > maxVelocity) {
            ratio = maxVelocity / norm;
        }
        return new FieldRelativeVelocity(ratio * x, ratio * y, MathUtil.clamp(theta, -maxOmega, maxOmega));
    }

    @Override
    public String toString() {
        return String.format("(%5.2f, %5.2f, %5.2f)", x, y, theta);
    }
}