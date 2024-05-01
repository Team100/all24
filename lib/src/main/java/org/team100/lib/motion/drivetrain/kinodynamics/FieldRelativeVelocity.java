package org.team100.lib.motion.drivetrain.kinodynamics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Just like ChassisSpeeds, but field-relative, to avoid mixing them up.
 */
public record FieldRelativeVelocity(double x, double y, double theta) {
    public double norm() {
        return Math.hypot(x, y);
    }

    public Rotation2d angle() {
        return new Rotation2d(x, y);
    }

    public FieldRelativeVelocity plus(FieldRelativeVelocity other) {
        return new FieldRelativeVelocity(x + other.x, y + other.y, theta + other.theta);
    }

    public FieldRelativeVelocity minus(FieldRelativeVelocity other) {
        return new FieldRelativeVelocity(x - other.x, y - other.y, theta - other.theta);
    }

    public FieldRelativeVelocity times(double scalar) {
        return new FieldRelativeVelocity(x * scalar, y * scalar, theta * scalar);
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
        return String.format("%s[x=%5.3f,y=%5.3f]", getClass().getSimpleName(), x, y);

    }
}