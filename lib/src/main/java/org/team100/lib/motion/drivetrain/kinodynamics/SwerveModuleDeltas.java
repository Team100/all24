package org.team100.lib.motion.drivetrain.kinodynamics;

/**
 * Container for swerve module deltas.
 * 
 * This is intended to avoid passing around an array of deltas,
 * and having to remember which location corresponds to which index.
 */
public record SwerveModuleDeltas(
        SwerveModuleDelta frontLeft,
        SwerveModuleDelta frontRight,
        SwerveModuleDelta rearLeft,
        SwerveModuleDelta rearRight) {
    /** For when you don't care about which is which. */
    public SwerveModuleDelta[] all() {
        return new SwerveModuleDelta[] {
                frontLeft,
                frontRight,
                rearLeft,
                rearRight
        };
    }
}
