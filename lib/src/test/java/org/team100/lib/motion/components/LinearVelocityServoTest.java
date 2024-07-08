package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.MockIncrementalLinearEncoder;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.TestLogger;

class LinearVelocityServoTest {
    private static final Logger logger = new TestLogger();

    @Test
    void testSimple() {
        MockBareMotor driveMotor = new MockBareMotor();
        MockIncrementalLinearEncoder driveEncoder = new MockIncrementalLinearEncoder();
        LinearMechanism mech = new LinearMechanism(driveMotor, 1, 1);
        OutboardLinearVelocityServo servo = new OutboardLinearVelocityServo(
                logger,
                mech,
                driveEncoder);
        // 0.5 m/s
        servo.setVelocity(0.5);
        // wheel radius is 0.5 m, so drive speed is 1 m/s
        assertEquals(1.0, driveMotor.velocity, 0.001);
    }
}
