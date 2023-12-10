package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Function;

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
 * 
 * This requires a controller under the manual command, so for now
 * it's not in use, but it should be.
 */
public class ManualSwerveState implements Function<Twist2d, SwerveState> {
    private static final double kScale = 1.0;

    @Override
    public SwerveState apply(Twist2d input) {
        return new SwerveState(
                new State100(kScale * input.dx, 0, 0),
                new State100(kScale * input.dy, 0, 0),
                new State100(kScale * input.dtheta, 0, 0));
    }
}
