package org.team100.lib.motion.crank;

import java.util.function.Supplier;

/** Marker for visible actuation supplier. */
public interface Actuations extends Supplier<Actuation>, Indicator.Visible {
}
