package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.testing.TimelessTest;
import org.team100.lib.units.Distance100;

class LimitedVelocityServoTest extends TimelessTest {
    private static final double kDelta = 0.0001;

    @Test
    void test100() {
        MockVelocityServo<Distance100> mv = new MockVelocityServo<>();
        double maxVel = 1;
        double maxAccel = 1;
        double maxDecel = -1;
        LimitedVelocityServo<Distance100> pv = new LimitedVelocityServo<>(
                mv, maxVel, maxAccel, maxDecel);

        assertEquals(0, mv.m_setpoint, kDelta);
        pv.reset();
        pv.setVelocity(1);
        // previous = 0, new = 1, time = 0, so new should be 0
        assertEquals(0, mv.m_setpoint, kDelta);

        stepTime(0.02);
        pv.setVelocity(1);
        // previous = 0, new = 1, time = 0.02, so new should be 0.02
        assertEquals(0.02, mv.m_setpoint, kDelta);

        stepTime(0.02);
        pv.setVelocity(1);
        // previous = 0.02, new = 1, time = 0.02, so new should be 0.04
        assertEquals(0.04, mv.m_setpoint, kDelta);
    }

}
