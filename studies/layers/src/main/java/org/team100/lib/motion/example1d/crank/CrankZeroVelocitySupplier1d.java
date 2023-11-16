package org.team100.lib.motion.example1d.crank;

import java.util.function.Supplier;

/**
 * Always supplies zero, for "safe mode."
 */
public class CrankZeroVelocitySupplier1d implements Supplier<CrankWorkstate> {

    @Override
    public CrankWorkstate get() {
        return new CrankWorkstate(0.0);
    }
}
