package org.team100.lib.motion.drivetrain.kinodynamics;

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
}