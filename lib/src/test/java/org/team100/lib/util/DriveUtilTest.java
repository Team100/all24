package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Twist2d;

class DriveUtilTest {
    private static final double kDelta = 0.001;

    @Test
    void testClampTwist() {
        {
            // zero is no-op
            Twist2d input = new Twist2d(0, 0, 0);
            Twist2d actual = DriveUtil.clampTwist(input, 1);
            assertEquals(0, actual.dx, kDelta);
            assertEquals(0, actual.dy, kDelta);
            assertEquals(0, actual.dtheta, kDelta);
        }
        {
            // clip to the unit circle.
            Twist2d input = new Twist2d(1, 1, 0);
            Twist2d actual = DriveUtil.clampTwist(input, 1);
            assertEquals(0.707, actual.dx, kDelta);
            assertEquals(0.707, actual.dy, kDelta);
        }
        {
            // leave the inside alone
            Twist2d input = new Twist2d(0.5, 0.5, 0);
            Twist2d actual = DriveUtil.clampTwist(input, 1);
            assertEquals(0.5, actual.dx, kDelta);
            assertEquals(0.5, actual.dy, kDelta);
        }
    }

}
