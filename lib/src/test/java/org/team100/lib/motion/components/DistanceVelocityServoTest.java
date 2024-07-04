package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.motor.MockVelocityMotor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.units.Distance100;

class Distance100VelocityServoTest {
    @Test
    void testSimple() {
        MockVelocityMotor100<Distance100> driveMotor = new MockVelocityMotor100<>();
        MockEncoder100<Distance100> driveEncoder = new MockEncoder100<>();
        Logger logger = Telemetry.get().testLogger();
        OutboardVelocityServo<Distance100> servo = new OutboardVelocityServo<>(
                logger,
                driveMotor,
                driveEncoder);
        servo.setVelocity(0.5);
        assertEquals(0.5, driveMotor.velocity, 0.001);
    }
}
