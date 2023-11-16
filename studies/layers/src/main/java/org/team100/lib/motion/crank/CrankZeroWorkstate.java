package org.team100.lib.motion.crank;

import java.util.function.Supplier;

/**
 * Always supplies zero, for "safe mode."
 */
public class CrankZeroWorkstate implements Supplier<CrankWorkstate> {

    @Override
    public CrankWorkstate get() {
        return new CrankWorkstate(0.0);
    }
}
