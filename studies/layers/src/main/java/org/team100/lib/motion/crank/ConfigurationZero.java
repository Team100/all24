package org.team100.lib.motion.crank;

import java.util.function.Supplier;

/** Always produces zero actuation. */
public class ConfigurationZero implements Supplier<Actuation> {

    @Override
    public Actuation get() {
        return new Actuation(0.0);
    }
}
