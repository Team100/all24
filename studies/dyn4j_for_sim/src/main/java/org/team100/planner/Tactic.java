package org.team100.planner;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

/**
 * Common interface for simple, stateless, tactical planners.
 */
public interface Tactic {
    /**
     * @param desired desired velocity, used to project future conflicts
     * @param debug   print details to the console
     * @return a feasible velocity
     */
    FieldRelativeVelocity apply(FieldRelativeVelocity desired);
}
