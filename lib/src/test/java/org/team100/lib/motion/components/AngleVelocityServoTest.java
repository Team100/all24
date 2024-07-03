package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.motor.MockVelocityMotor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.units.Angle100;

class AngleVelocityServoTest {
    @Test
    void testOutboard() {
        String name = "test";
        MockVelocityMotor100<Angle100> motor = new MockVelocityMotor100<>();
        MockEncoder100<Angle100> encoder = new MockEncoder100<>();
        Logger logger = Telemetry.get().rootLogger("foo");
        OutboardVelocityServo<Angle100> servo = new OutboardVelocityServo<>(
                name,
                logger,
                motor,
                encoder);

        servo.setVelocity(0.5);
        assertEquals(0.5, motor.velocity, 0.001);
    }
}
