package org.team100.lib.motion.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SpeedLimits;

import edu.wpi.first.math.geometry.Twist2d;

public class ManualFieldRelativeSpeedsTest {
    private static final double kDelta = 0.001;

    @Test
    void testTwistZero() {
        Supplier<Twist2d> input = () -> new Twist2d();
        SpeedLimits limits = new SpeedLimits(1, 1, 1, 1);
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(input, limits);
        Twist2d twist = manual.get();
        assertEquals(0, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0, twist.dtheta, kDelta);
    }

    @Test
    void testTwistNonzero() {
        Supplier<Twist2d> input = () -> new Twist2d(1, 2, 3);
        SpeedLimits limits = new SpeedLimits(1, 1, 1, 1);
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(input, limits);
        Twist2d twist = manual.get();
        assertEquals(1, twist.dx, kDelta);
        assertEquals(1, twist.dy, kDelta); // speed limit
        assertEquals(1, twist.dtheta, kDelta); // speed limit
    }

}
