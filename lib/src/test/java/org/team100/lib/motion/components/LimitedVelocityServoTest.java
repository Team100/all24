package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.util.WPIUtilJNI;

class LimitedVelocityServoTest {
    private static final double kDelta = 0.0001;

    @Test
    void test100() {
        MockVelocityServo<Distance> mv = new MockVelocityServo<>();
        double maxVel = 1;
        double maxAccel = 1;
        LimitedVelocityServo<Distance> pv = new LimitedVelocityServo<>(mv, maxVel, maxAccel);

        // slew rate limiter tells time via
        // WPIUtilJNI.now() rather than FPGA.getTimestamp,
        // so for testing you use this WPIUtilJNI mock time thing.

        WPIUtilJNI.enableMockTime();
        WPIUtilJNI.setMockTime(0);
        assertEquals(0, MathSharedStore.getTimestamp());
        assertEquals(0, mv.m_setpoint, kDelta);
        pv.reset();
        pv.setVelocity(1);
        // previous = 0, new = 1, time = 0, so new should be 0
        assertEquals(0, mv.m_setpoint, kDelta);

        WPIUtilJNI.setMockTime(20000);
        pv.setVelocity(1);
        // previous = 0, new = 1, time = 0.02, so new should be 0.02
        assertEquals(0.02, mv.m_setpoint, kDelta);

        WPIUtilJNI.setMockTime(40000);
        pv.setVelocity(1);
        // previous = 0.02, new = 1, time = 0.02, so new should be 0.04
        assertEquals(0.04, mv.m_setpoint, kDelta);
    }

}
