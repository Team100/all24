package org.team100.lib.encoder.drive;

public interface DriveEncoder {
    /** @return encoder rate in meters per second */
    double getRate();

    /** @return accumulated distance in meters */
    double getDistance();

    /** set distance to zero */
    void reset();
}