package org.team100.lib.motion.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;


import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Twist2d;

public class ManualSwerveStateTest {
    private static final double kDelta = 0.001;

    @Test
    void testSwerveStateZero() {
        ManualSwerveState manual = new ManualSwerveState();
        Twist2d input = new Twist2d();
        SwerveState state = manual.apply(input);
        assertEquals(0, state.x().x(), kDelta);
        assertEquals(0, state.y().x(), kDelta);
        assertEquals(0, state.theta().x(), kDelta);
    }

    @Test
    void testSwerveStateNonzero() {
        ManualSwerveState manual = new ManualSwerveState();
        Twist2d input = new Twist2d(1, 2, 3);
        SwerveState state = manual.apply(input);
        assertEquals(1, state.x().x(), kDelta);
        assertEquals(2, state.y().x(), kDelta);
        assertEquals(3, state.theta().x(), kDelta);
    }
}
