package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SpeedLimits {
    public final double speedM_S;
    public final double accelM_S2;
    public final double angleSpeedRad_S;
    public final double angleAccelRad_S2;

    public SpeedLimits(
            double speedM_S,
            double accelM_S2,
            double angleSpeedRad_S,
            double angleAccelRad_S2) {
        this.speedM_S = speedM_S;
        this.accelM_S2 = accelM_S2;
        this.angleSpeedRad_S = angleSpeedRad_S;
        this.angleAccelRad_S2 = angleAccelRad_S2;
    }

    public TrapezoidProfile.Constraints constraints() {
        return new TrapezoidProfile.Constraints(angleSpeedRad_S, angleAccelRad_S2);
    }
}
