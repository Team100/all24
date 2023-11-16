package org.team100.lib.motion.example1d.crank;

import java.util.function.Supplier;

/** Always produces zero actuation. */
public class CrankConfigurationZero implements Supplier<CrankActuation> {

    @Override
    public CrankActuation get() {
        return new CrankActuation(0.0);
    }
}
