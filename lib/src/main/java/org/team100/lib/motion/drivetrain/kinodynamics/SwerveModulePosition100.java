package org.team100.lib.motion.drivetrain.kinodynamics;

import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;
import java.util.Optional;

import org.team100.lib.motion.drivetrain.kinodynamics.struct.SwerveModulePosition100Struct;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * This is a copy of {@link edu.wpi.first.math.kinematics.SwerveModulePosition}
 * but with optional rotation, working around the incorrect behavior of
 * Rotation2d(0, 0).
 * 
 * Represents the state of one swerve module.
 */
public class SwerveModulePosition100
        implements Comparable<SwerveModulePosition100>,
        Interpolatable<SwerveModulePosition100>,
        StructSerializable {
    /** Distance measured by the wheel of the module. */
    public double distanceMeters;

    /**
     * Angle of the module. It can be empty, in cases where the angle is
     * indeterminate (e.g. calculating the angle required for zero speed).
     * TODO: make this private.
     */
    public Optional<Rotation2d> angle = Optional.empty();

    /** SwerveModulePosition struct for serialization. */
    public static final SwerveModulePosition100Struct struct = new SwerveModulePosition100Struct();

    /** Constructs a SwerveModulePosition with zeros for distance and angle. */
    public SwerveModulePosition100() {
    }

    /**
     * Constructs a SwerveModulePosition.
     *
     * @param distanceMeters The distance measured by the wheel of the module.
     * @param angle          The angle of the module.
     */
    public SwerveModulePosition100(double distanceMeters, Optional<Rotation2d> angle) {
        this.distanceMeters = distanceMeters;
        this.angle = angle;
    }

    /**
     * Constructs a SwerveModulePosition.
     *
     * @param distance The distance measured by the wheel of the module.
     * @param angle    The angle of the module.
     */
    public SwerveModulePosition100(Measure<Distance> distance, Optional<Rotation2d> angle) {
        this(distance.in(Meters), angle);
    }

    /** */
    public SwerveModulePosition100(double x, double y) {
        if (Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6) {
            // avoid the garbage rotation.
            this.distanceMeters = 0.0;
            this.angle = Optional.empty();
        } else {
            this.distanceMeters = Math.hypot(x, y);
            this.angle = Optional.of(new Rotation2d(x, y));
        }
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof SwerveModulePosition100 other
                && Math.abs(other.distanceMeters - distanceMeters) < 1E-9
                && angle.equals(other.angle);
    }

    @Override
    public int hashCode() {
        return Objects.hash(distanceMeters, angle);
    }

    /**
     * Compares two swerve module positions. One swerve module is "greater" than the
     * other if its
     * distance is higher than the other.
     *
     * @param other The other swerve module.
     * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
     */
    @Override
    public int compareTo(SwerveModulePosition100 other) {
        return Double.compare(this.distanceMeters, other.distanceMeters);
    }

    @Override
    public String toString() {
        return String.format(
                "SwerveModulePosition(Distance: %.2f m, Angle: %s)", distanceMeters, angle);
    }

    /**
     * Returns a copy of this swerve module position.
     *
     * @return A copy.
     */
    public SwerveModulePosition100 copy() {
        return new SwerveModulePosition100(distanceMeters, angle);
    }

    @Override
    public SwerveModulePosition100 interpolate(SwerveModulePosition100 endValue, double t) {
        double distLerp = MathUtil.interpolate(this.distanceMeters, endValue.distanceMeters, t);
        if (this.angle.isEmpty() || endValue.angle.isEmpty()) {
            return new SwerveModulePosition100(distLerp, Optional.empty());
        } else {
            Rotation2d angleLerp = this.angle.get().interpolate(endValue.angle.get(), t);
            return new SwerveModulePosition100(distLerp, Optional.of(angleLerp));
        }
    }
}
