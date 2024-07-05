package org.team100.lib.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motor.drive.DriveMotorController100;
import org.team100.lib.motor.turning.TurningMotorController100;
import org.team100.lib.telemetry.TestLogger;
import org.team100.lib.telemetry.Logger;

class MotorController100Test {
    private static final double kDelta = 0.001;
    private static final Logger logger = new TestLogger();

    @Test
    void testTurning() {
        MockMotorController mc = new MockMotorController();
        TurningMotorController100 m = new TurningMotorController100(logger, mc, 2);
        m.setVelocity(0, 0, 0);
        assertEquals(0, mc.speed, kDelta);
        m.setVelocity(1, 0, 0);
        assertEquals(0.0032, mc.speed, kDelta);
    }

    @Test
    void testSimple() {
        MockMotorController mc = new MockMotorController();
        DriveMotorController100 m = new DriveMotorController100(logger, mc, 2, 0.1);
        m.setVelocity(0, 0, 0);
        assertEquals(0, mc.speed, kDelta);
        m.setVelocity(1, 0, 0);
        assertEquals(0.064, mc.speed, kDelta);
    }
}
