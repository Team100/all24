package org.team100.lib.motion.drivetrain.kinodynamics;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Container for swerve module headings.
 * 
 * This is so you don't have to remember which location is which index.
 * 
 * Fields are nullable.
 */
public record SwerveModuleHeadings(
        Rotation2d frontLeft,
        Rotation2d frontRight,
        Rotation2d rearLeft,
        Rotation2d rearRight) {
}
