package org.team100.lib.planner;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

/**
 * Common interface for simple, stateless, tactical planners.
 */
public interface Tactic {
    /**
     * TODO: maybe this should operate on forces instead of velocities?
     * 
     * @param desired desired velocity, used to project future conflicts
     * @return a feasible velocity
     */
    FieldRelativeVelocity apply(FieldRelativeVelocity desired);
}