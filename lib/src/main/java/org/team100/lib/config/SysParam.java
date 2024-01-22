package org.team100.lib.config;

/**
 * TODO(sanjan): make several of these sysparam classes, one per subsystem,
 * instead of using optional parameters.
 * 
 * @param gearRatio
 * @param wheelDiameter
 * @param maxVelM_S
 * @param maxAccelM_S2
 * @param maxDecelM_S2 must be non-positive.
 */
public record SysParam(
        double gearRatio,
        double wheelDiameter,
        double maxVelM_S,
        double maxAccelM_S2,
        double maxDecelM_S2) {
    // No need to explicitly declare a constructor or getter/setter methods

    public static SysParam limitedNeoVelocityServoSystem(double gearRatio, double wheelDiameter, double maxVel,
            double maxAccel, double maxDecel) {
        return new SysParam(gearRatio, wheelDiameter, maxVel, maxAccel, maxDecel);
    }

    public static SysParam neoVelocityServoSystem(double gearRatio, double wheelDiameter) {
        // Assuming a default value for kWheelDiameter in the intake configuration
        return new SysParam(gearRatio, wheelDiameter, 0, 0, 0);
    }

    public static SysParam neoPositionServoSystem(double gearRatio, double maxVel,
            double maxAccel) {
        return new SysParam(gearRatio, 0, maxVel, maxAccel, 0);
    }
}
