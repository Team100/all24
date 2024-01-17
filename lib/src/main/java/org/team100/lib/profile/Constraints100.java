package org.team100.lib.profile;

/**
 * These constraints should also include deceleration, which can be different
 * (higher) than acceleration.
 */
public class Constraints100 {
    public final double maxVelocity;

    public final double maxAcceleration;

    public Constraints100(
            double maxVelocity,
            double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }
}