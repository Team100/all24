package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Supplier;

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
 * and speed, with a deadband in the center.  The input twist dtheta is ignored.
 */
public class ManualModuleStates implements Supplier<SwerveModuleState[]> {
    private static final double kDeadband = 0.2;
    private static final double kScale = 1.0;
    private final Supplier<Twist2d> m_input;

    public ManualModuleStates(Supplier<Twist2d> input) {
        m_input = input;
    }

    @Override
    public SwerveModuleState[] get() {
        Twist2d input = m_input.get();
        double hyp = Math.hypot(input.dx, input.dy);
        double speedM_S = 0.0;
        Rotation2d angle = new Rotation2d();
        if (hyp >= kDeadband) {
            speedM_S = kScale * MathUtil.applyDeadband(hyp, kDeadband);
            angle = new Rotation2d(input.dx, input.dy);
        }
        return new SwerveModuleState[] {
                new SwerveModuleState(speedM_S, angle),
                new SwerveModuleState(speedM_S, angle),
                new SwerveModuleState(speedM_S, angle),
                new SwerveModuleState(speedM_S, angle)
        };
    }
}
