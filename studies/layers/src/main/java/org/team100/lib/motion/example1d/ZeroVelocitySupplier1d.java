package org.team100.lib.motion.example1d;

import java.util.function.DoubleSupplier;

/** Always supplies zero, for "safe mode."" */
public class ZeroVelocitySupplier1d implements DoubleSupplier {

    /** @return velocity in meters per second */
    @Override
    public double getAsDouble() {
        return 0.0;
    }
}
