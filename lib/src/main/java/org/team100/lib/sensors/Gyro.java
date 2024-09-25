package org.team100.lib.sensors;

import org.team100.lib.dashboard.Glassy;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Three-axis gyro, NWU.
 * 
 * Implementations should extrapolate measurements to the current instant.
 * 
 * TODO: make all these optionals in case the gyro fails
 * TODO: maybe include timestamp here somehow?
 */
public interface Gyro extends Glassy {
    /** Yaw in radians, NWU, counterclockwise positive. */
    Rotation2d getYawNWU();

    /** Yaw rate in rad/s, NWU, counterclockwise positive. */
    double getYawRateNWU();

    /** Pitch in radians, NWU, positive-down. */
    Rotation2d getPitchNWU();

    /** Roll in radians, NWU, positive-right. */
    Rotation2d getRollNWU();

}
