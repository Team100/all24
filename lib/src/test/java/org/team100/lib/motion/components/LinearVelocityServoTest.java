package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.TestLogger;

class LinearVelocityServoTest {
    private static final SupplierLogger logger = new TestLogger().getSupplierLogger();

    @Test
    void testSimple() {
        MockBareMotor driveMotor = new MockBareMotor();
        MockIncrementalBareEncoder driveEncoder = new MockIncrementalBareEncoder();
        LinearMechanism mech = new LinearMechanism(driveMotor, driveEncoder, 1, 1);
        OutboardLinearVelocityServo servo = new OutboardLinearVelocityServo(
                logger,
                mech);
        // 0.5 m/s
        servo.setVelocity(0.5);
        // wheel radius is 0.5 m, so drive speed is 1 m/s
        assertEquals(1.0, driveMotor.velocity, 0.001);
    }
}
