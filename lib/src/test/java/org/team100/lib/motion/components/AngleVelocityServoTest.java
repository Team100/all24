package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.motor.MockVelocityMotor100;
import org.team100.lib.telemetry.TestLogger;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.units.Angle100;

class AngleVelocityServoTest {
    private static final Logger logger = new TestLogger();

    @Test
    void testOutboard() {
        MockVelocityMotor100<Angle100> motor = new MockVelocityMotor100<>();
        MockEncoder100<Angle100> encoder = new MockEncoder100<>();
        OutboardVelocityServo<Angle100> servo = new OutboardVelocityServo<>(
                logger,
                motor,
                encoder);
        servo.setVelocity(0.5);
        assertEquals(0.5, motor.velocity, 0.001);
    }
}
