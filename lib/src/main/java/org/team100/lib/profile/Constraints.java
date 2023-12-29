package org.team100.lib.profile;

public class Constraints {
    public final double maxVelocity;

    public final double maxAcceleration;

    /**
     * Construct constraints for a TrapezoidProfile.
     *
     * @param maxVelocity     maximum velocity
     * @param maxAcceleration maximum acceleration
     */
    public Constraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }
}