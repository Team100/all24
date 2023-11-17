package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Transform manual input into a desired SwerveState.
 * 
 * The three components of twist are mapped to absolute states,
 * with zero velocity/acceleration.
 * 
 * Be careful about where the "zero" is.
 */
public class ManualSwerveState implements Supplier<SwerveState> {
    private static final double kScale = 1.0;
    private final Supplier<Twist2d> m_input;

    public ManualSwerveState(Supplier<Twist2d> input) {
        m_input = input;
    }

    @Override
    public SwerveState get() {
        Twist2d input = m_input.get();
        return new SwerveState(
                new State100(kScale * input.dx, 0, 0),
                new State100(kScale * input.dy, 0, 0),
                new State100(kScale * input.dtheta, 0, 0));
    }
}
