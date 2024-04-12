package org.team100.lib.config;


public record SysParam(
        double gearRatio,
        double wheelDiameter,
        double maxVelM_S,
        double maxAccelM_S2,
        double maxDecelM_S2) {

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
