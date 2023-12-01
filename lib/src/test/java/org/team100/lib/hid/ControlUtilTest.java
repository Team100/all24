package org.team100.lib.hid;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Twist2d;

public class ControlUtilTest {
    private static final double kDelta = 0.001;

    @Test
    void testExpo() {
        assertEquals(0.3125, ControlUtil.expo(0.5, 0.5), kDelta);
    }

    @Test
    void testDeadband() {
        assertEquals(0, ControlUtil.deadband(0.1, 0.2, 1.0), kDelta);

    }

    @Test
    void testClamp() {
        assertEquals(0.5, ControlUtil.clamp(2, 0.5), kDelta);
    }

    @Test
    void testClampTwist() {
        {
            Twist2d input = new Twist2d(1, 1, 0);
            Twist2d actual = ControlUtil.clampTwist(input, 1);
            assertEquals(0.707, actual.dx, kDelta);
            assertEquals(0.707, actual.dy, kDelta);
        }
       {
            Twist2d input = new Twist2d(0.5, 0.5, 0);
            Twist2d actual = ControlUtil.clampTwist(input, 1);
            assertEquals(0.5, actual.dx, kDelta);
            assertEquals(0.5, actual.dy, kDelta);
        }
    }

}
