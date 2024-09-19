package org.team100.lib.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.TestLogger;

class CombinedEncoderTest {
    private static final double kDelta = 0.001;
    private static final SupplierLogger logger = new TestLogger().getSupplierLogger();

    // TODO: FIX THIS TEST
    // @Test
    void testSimple1() {
        MockRotaryPositionSensor e1 = new MockRotaryPositionSensor();
        MockBareMotor motor = new MockBareMotor();
        MockIncrementalBareEncoder e2 = new MockIncrementalBareEncoder();
        RotaryMechanism m = new RotaryMechanism(logger, motor, e2, 1.0);
        CombinedEncoder c = new CombinedEncoder(logger, e1, 1.0, m);
        e1.angle = 1; // this is the authority
        e2.position = 0; // this is wrong
        // read the authority value
        assertEquals(1.0, c.getPositionRad().getAsDouble(), kDelta);
        // and fix the secondary
        assertEquals(1.0, e2.position, kDelta);
    }

    // TODO: FIX THIS TEST
    // @Test
    void testHalfPrimary() {
        // primary, absolute sensor
        MockRotaryPositionSensor e1 = new MockRotaryPositionSensor();
        MockBareMotor motor = new MockBareMotor();
        // secondary, built-in encoder.
        MockIncrementalBareEncoder e2 = new MockIncrementalBareEncoder();
        RotaryMechanism m = new RotaryMechanism(logger, motor, e2, 1.0);
        CombinedEncoder c = new CombinedEncoder(logger, e1, 0.5, m);
        e1.angle = 1; // this is the authority
        e2.position = 0; // this is wrong
        // read the authority value
        assertEquals(0.5, c.getPositionRad().getAsDouble(), kDelta);
        // and fix the secondary
        assertEquals(0.5, e2.position, kDelta);
    }

    @Test
    void testIgnorePrimary() {
        MockRotaryPositionSensor e1 = new MockRotaryPositionSensor();
        MockBareMotor motor = new MockBareMotor();
        MockIncrementalBareEncoder e2 = new MockIncrementalBareEncoder();
        RotaryMechanism m = new RotaryMechanism(logger, motor, e2, 1.0);
        CombinedEncoder c = new CombinedEncoder(logger, e1, 0.0, m);
        e1.angle = 1; // this is the authority
        e2.position = 0; // this is wrong
        // read the authority value
        assertEquals(0.0, c.getPositionRad().getAsDouble(), kDelta);
        // and fix the secondary
        assertEquals(0.0, e2.position, kDelta);
    }
}
