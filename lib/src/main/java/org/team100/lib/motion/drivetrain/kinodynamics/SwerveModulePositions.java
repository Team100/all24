package org.team100.lib.motion.drivetrain.kinodynamics;

/**
 * Container for swerve module positions.
 * 
 * This is intended to avoid passing around an array of positions,
 * and having to remember which location corresponds to which index.
 */
public record SwerveModulePositions(
        SwerveModulePosition100 frontLeft,
        SwerveModulePosition100 frontRight,
        SwerveModulePosition100 rearLeft,
        SwerveModulePosition100 rearRight) {
    /** For when you don't care about which is which. */
    public SwerveModulePosition100[] all() {
        return new SwerveModulePosition100[] {
                frontLeft,
                frontRight,
                rearLeft,
                rearRight
        };
    }

    public SwerveModulePositions(SwerveModulePositions other) {
        this(other.frontLeft.copy(),
                other.frontRight.copy(),
                other.rearLeft.copy(),
                other.rearRight.copy());
    }
}
