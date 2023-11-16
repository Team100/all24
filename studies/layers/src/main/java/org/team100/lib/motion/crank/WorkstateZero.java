package org.team100.lib.motion.crank;

import java.util.function.Supplier;

/**
 * Always supplies zero, for "safe mode."
 */
public class WorkstateZero implements Supplier<Workstate> {

    @Override
    public Workstate get() {
        return new Workstate(0.0);
    }
}
