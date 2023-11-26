package org.team100.lib.swerve;

/**
 * This is from 254 2023
 * TODO: move this to SpeedLimits
 */
public class SwerveKinematicLimits {
    public final double kMaxDriveVelocity; // m/s
    public final double kMaxDriveAcceleration; // m/s^2
    public final double kMaxSteeringVelocity; // rad/s

    public SwerveKinematicLimits(
            double kMaxDriveVelocity,
            double kMaxDriveAcceleration,
            double kMaxSteeringVelocity) {
        this.kMaxDriveVelocity = kMaxDriveVelocity;
        this.kMaxDriveAcceleration = kMaxDriveAcceleration;
        this.kMaxSteeringVelocity = kMaxSteeringVelocity;
    }

}
