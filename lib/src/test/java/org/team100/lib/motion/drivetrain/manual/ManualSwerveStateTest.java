package org.team100.lib.motion.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Twist2d;

public class ManualSwerveStateTest {
    private static final double kDelta = 0.001;

    @Test
    void testSwerveStateZero() {
        Supplier<Twist2d> input = () -> new Twist2d();
        ManualSwerveState manual = new ManualSwerveState(input);
        SwerveState state = manual.get();
        assertEquals(0, state.x().x(), kDelta);
        assertEquals(0, state.y().x(), kDelta);
        assertEquals(0, state.theta().x(), kDelta);
    }

    @Test
    void testSwerveStateNonzero() {
        Supplier<Twist2d> input = () -> new Twist2d(1, 2, 3);
        ManualSwerveState manual = new ManualSwerveState(input);
        SwerveState state = manual.get();
        assertEquals(1, state.x().x(), kDelta);
        assertEquals(2, state.y().x(), kDelta);
        assertEquals(3, state.theta().x(), kDelta);
    }
}
