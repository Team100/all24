package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Function;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Transform manual input into swerve module states.
 * 
 * Every module state is set the same, resulting in a "crab drive" that can
 * strafe but not rotate.
 * 
 * Module states are two-dimensional: steering angle and drive speed.
 * 
 * The x and y components of the input twist are used to determine both angle
 * and speed, with a deadband in the center. The input twist dtheta is ignored.
 */
public class ManualModuleStates implements Function<Twist2d, SwerveModuleState[]> {
    private final Telemetry t = Telemetry.get();
    private static final double kDeadband = 0.2;
    private final SpeedLimits m_speedLimits;

    public ManualModuleStates(SpeedLimits speedLimits) {
        m_speedLimits = speedLimits;
    }

    @Override
    public SwerveModuleState[] apply(Twist2d input) {
        double hyp = Math.hypot(input.dx, input.dy);
        hyp = MathUtil.clamp(hyp, 0, 1);
        double speedM_S = 0.0;
        Rotation2d angle = GeometryUtil.kRotationZero;
        if (hyp >= kDeadband) {
            speedM_S = m_speedLimits.speedM_S * MathUtil.applyDeadband(hyp, kDeadband, 1);
            angle = new Rotation2d(input.dx, input.dy);
        }
        t.log(Level.DEBUG, "/manual module state/v m_s", speedM_S);
        t.log(Level.DEBUG, "/manual module state/angle rad", angle.getRadians());
        return new SwerveModuleState[] {
                new SwerveModuleState(speedM_S, angle),
                new SwerveModuleState(speedM_S, angle),
                new SwerveModuleState(speedM_S, angle),
                new SwerveModuleState(speedM_S, angle)
        };
    }
}
