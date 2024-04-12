package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.hid.DriverControl;

class DriveUtilTest {
    private static final double kDelta = 0.001;

    @Test
    void testClampTwist() {
        {
            // zero is no-op
            DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
            DriverControl.Velocity actual = DriveUtil.clampTwist(input, 1);
            assertEquals(0, actual.x(), kDelta);
            assertEquals(0, actual.y(), kDelta);
            assertEquals(0, actual.theta(), kDelta);
        }
        {
            // clip to the unit circle.
            DriverControl.Velocity input = new DriverControl.Velocity(1, 1, 0);
            DriverControl.Velocity actual = DriveUtil.clampTwist(input, 1);
            assertEquals(0.707, actual.x(), kDelta);
            assertEquals(0.707, actual.y(), kDelta);
        }
        {
            // leave the inside alone
            DriverControl.Velocity input = new DriverControl.Velocity(0.5, 0.5, 0);
            DriverControl.Velocity actual = DriveUtil.clampTwist(input, 1);
            assertEquals(0.5, actual.x(), kDelta);
            assertEquals(0.5, actual.y(), kDelta);
        }
    }

}
