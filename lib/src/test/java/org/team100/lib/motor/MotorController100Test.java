package org.team100.lib.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.TestLogger;
import org.team100.lib.logging.SupplierLogger;

class MotorController100Test {
    private static final double kDelta = 0.001;
    private static final SupplierLogger logger = new TestLogger().getSupplierLogger();

    @Test
    void testTurning() {
        MockMotorController mc = new MockMotorController();
        BareMotorController100 m = new BareMotorController100(logger, mc);
        m.setVelocity(0, 0, 0);
        assertEquals(0, mc.speed, kDelta);
        m.setVelocity(1, 0, 0);
        assertEquals(0.0016, mc.speed, kDelta);
    }
}
