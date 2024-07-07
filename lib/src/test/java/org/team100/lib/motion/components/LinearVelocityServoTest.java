package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.MockIncrementalLinearEncoder;
import org.team100.lib.motor.MockLinearVelocityMotor100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.TestLogger;

class LinearVelocityServoTest {
    private static final Logger logger = new TestLogger();

    @Test
    void testSimple() {
        MockLinearVelocityMotor100 driveMotor = new MockLinearVelocityMotor100();
        MockIncrementalLinearEncoder driveEncoder = new MockIncrementalLinearEncoder();
        OutboardLinearVelocityServo servo = new OutboardLinearVelocityServo(
                logger,
                driveMotor,
                driveEncoder);
        servo.setVelocity(0.5);
        assertEquals(0.5, driveMotor.velocity, 0.001);
    }
}
