package org.team100.lib.sensors;

import org.team100.lib.dashboard.Glassy;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * This makes testing easier.
 * 
 * Methods return NWU, counterclockwise positive.
 */
public interface HeadingInterface extends Glassy {
    /**
     * NWU coordinates in rad: counterclockwise positive.
     */
    Rotation2d getHeadingNWU();

    /**
     * Rate in rad/s: counterclockwise positive.
     * 
     * This should apply whatever deadbanding should be used for the implementing
     * sensor.
     */
    double getHeadingRateNWU();

    @Override
    default String getGlassName() {
        return "Heading";
    }

}
