package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

/** This makes testing easier.  NWU, counterclockwise positive. */
public interface HeadingInterface {
    /** NWU coordinates in rad: counterclockwise positive. */
    Rotation2d getHeadingNWU();
    /** Rate in rad/s: counterclockwise positive. */
    double getHeadingRateNWU();
}
