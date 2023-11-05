package org.team100.lib.profile;

/**
 * Motion profile velocity constraint.
 */
public interface VelocityConstraint {
    /**
     * Returns the maximum profile velocity at displacement [s].
     */
    double get(double s);
}
