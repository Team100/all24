package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.MockEncoder100;
import org.team100.lib.motor.MockLinearVelocityMotor100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.TestLogger;
import org.team100.lib.units.Distance100;

class LinearVelocityServoTest {
    private static final Logger logger = new TestLogger();

    @Test
    void testSimple() {
        MockLinearVelocityMotor100 driveMotor = new MockLinearVelocityMotor100();
        MockEncoder100<Distance100> driveEncoder = new MockEncoder100<>();
        OutboardLinearVelocityServo servo = new OutboardLinearVelocityServo(
                logger,
                driveMotor,
                driveEncoder);
        servo.setVelocity(0.5);
        assertEquals(0.5, driveMotor.velocity, 0.001);
    }
}
