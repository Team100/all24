package org.team100.lib.sensors;

import org.team100.lib.dashboard.Glassy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This makes testing easier.
 * 
 * Methods return NWU, counterclockwise positive.
 */
public interface HeadingInterface extends Subsystem, Glassy {
    /**
     * NWU coordinates in rad: counterclockwise positive.
     */
    Rotation2d getHeadingNWU();

    /**
     * Rate in rad/s: counterclockwise positive.
     */
    double getHeadingRateNWU();

    @Override
    default String getGlassName() {
        return "Heading";
    }

}
