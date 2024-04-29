package org.team100.lib.motion.drivetrain.kinodynamics;

public record FieldRelativeAcceleration(double x, double y, double theta) {
    public FieldRelativeVelocity integrate(double dtSec) {
        return new FieldRelativeVelocity(x * dtSec, y * dtSec, theta * dtSec);
    }

    public FieldRelativeAcceleration plus(FieldRelativeAcceleration other) {
        return new FieldRelativeAcceleration(x + other.x, y + other.y, theta + other.theta);
    }

    public FieldRelativeAcceleration times(double scalar) {
        return new FieldRelativeAcceleration(x * scalar, y * scalar, theta * scalar);
    }

    public static FieldRelativeAcceleration diff(
            FieldRelativeVelocity v1,
            FieldRelativeVelocity v2,
            double dtSec) {
        FieldRelativeVelocity dv = v2.minus(v1);
        return new FieldRelativeAcceleration(dv.x() / dtSec, dv.y() / dtSec, dv.theta() / dtSec);

    }
}
