package org.team100.lib.sensors;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * This makes testing easier.
 * 
 * Methods return NWU, counterclockwise positive.
 */
public interface HeadingInterface {
    /** NWU coordinates in rad: counterclockwise positive. */
    Rotation2d getHeadingNWU();

    /** Rate in rad/s: counterclockwise positive. */
    double getHeadingRateNWU();
}
