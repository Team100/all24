package org.team100.lib.config;

/**
 * TODO(sanjan) I think this would be nice and clean as a "record"
 * https://www.baeldung.com/java-record-keyword
 * 
 * TODO(sanjan): make several of these sysparam classes, one per subsystem,
 * instead of using optional parameters.
 */

public record SysParam(
        double kGearRatio,
        double kWheelDiameter,
        double kMaxVelM_S,
        double kMaxAccelM_S2,
        double kMaxDecel) {
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
