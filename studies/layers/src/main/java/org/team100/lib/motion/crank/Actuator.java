package org.team100.lib.motion.crank;

import java.util.function.Consumer;

/** Marker for visible actuator. */
public interface Actuator extends Consumer<Actuation>, Indicator.Visible {
}
