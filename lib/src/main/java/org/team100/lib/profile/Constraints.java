package org.team100.lib.profile;

/**
 * These constraints should also include deceleration, which can be different
 * (higher) than acceleration.
 */
public class Constraints {
    public final double maxVelocity;

    public final double maxAcceleration;

    public final double maxJerk;

    /**
     * 
     * @param maxVelocity
     * @param maxAcceleration
     * @param maxJerk use Double.POSITIVE_INFINITY if you don't want to limit jerk.
     */
    public Constraints(
            double maxVelocity,
            double maxAcceleration,
            double maxJerk) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxJerk = maxJerk;
    }
}