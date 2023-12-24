package org.team100.lib.motion.drivetrain.manual;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Transform manual input into swerve module states, in a simpler way than
 * ManualModuleStates does.
 * 
 * The input dtheta is exactly the module angle.
 * The input dx is exactly the wheel speed.
 * The input dy is ignored.
 */
public class SimpleManualModuleStates {
    private final Telemetry t = Telemetry.get();
    private final SwerveKinodynamics m_speedLimits;

    public SimpleManualModuleStates(SwerveKinodynamics speedLimits) {
        m_speedLimits = speedLimits;
    }

    public SwerveModuleState[] apply(Twist2d input) {
        // dtheta is from [-1, 1], so angle is [-pi, pi]
        Rotation2d angle = Rotation2d.fromRadians(Math.PI * input.dtheta);
        double speedM_S = m_speedLimits.getMaxSpeedM_S() * input.dx;
        t.log(Level.DEBUG, "/simple manual module state/v m_s", speedM_S);
        t.log(Level.DEBUG, "/simple manual module state/angle rad", angle.getRadians());
        return new SwerveModuleState[] {
                new SwerveModuleState(speedM_S, angle),
                new SwerveModuleState(speedM_S, angle),
                new SwerveModuleState(speedM_S, angle),
                new SwerveModuleState(speedM_S, angle)
        };
    }
}
