package org.team100.lib.swerve;

/**
 * This represents acceleration and deceleration separately since motors are
 * better at slowing down than they are at speeding up.
 * 
 * This is from 254 2023
 * TODO: move this to SpeedLimits
 */
public class SwerveKinematicLimits {
    public final double kMaxDriveVelocity; // m/s
    public final double kMaxDriveAcceleration; // m/s^2
    public final double kMaxDriveDeceleration; // m/s^2
    public final double kMaxSteeringVelocity; // rad/s
    public final double kMaxCentripetalAccel; // m/s^2

    public SwerveKinematicLimits(
            double maxDriveVelocity,
            double maxDriveAcceleration,
            double maxDriveDeceleration,
            double maxSteeringVelocity,
            double maxCentripetalAccel) {
        kMaxDriveVelocity = maxDriveVelocity;
        kMaxDriveAcceleration = maxDriveAcceleration;
        kMaxDriveDeceleration = maxDriveDeceleration;
        kMaxSteeringVelocity = maxSteeringVelocity;
        kMaxCentripetalAccel = maxCentripetalAccel;
    }

}
