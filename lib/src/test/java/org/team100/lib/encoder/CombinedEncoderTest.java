package org.team100.lib.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class CombinedEncoderTest {
    private static final double kDelta = 0.001;

    @Test
    void testSimple1() {
        MockRotaryPositionSensor e1 = new MockRotaryPositionSensor();
        MockSettableAngularEncoder e2 = new MockSettableAngularEncoder();
        CombinedEncoder c = new CombinedEncoder(e1, 1.0, e2);
        e1.angle = 1; // this is the authority
        e2.angle = 0; // this is wrong
        // read the authority value
        assertEquals(1.0, c.getPositionRad().getAsDouble(), kDelta);
        // and fix the secondary
        assertEquals(1.0, e2.angle, kDelta);
    }

    @Test
    void testHalfPrimary() {
        MockRotaryPositionSensor e1 = new MockRotaryPositionSensor();
        MockSettableAngularEncoder e2 = new MockSettableAngularEncoder();
        CombinedEncoder c = new CombinedEncoder(e1, 0.5, e2);
        e1.angle = 1; // this is the authority
        e2.angle = 0; // this is wrong
        // read the authority value
        assertEquals(0.5, c.getPositionRad().getAsDouble(), kDelta);
        // and fix the secondary
        assertEquals(0.5, e2.angle, kDelta);
    }

    @Test
    void testIgnorePrimary() {
        MockRotaryPositionSensor e1 = new MockRotaryPositionSensor();
        MockSettableAngularEncoder e2 = new MockSettableAngularEncoder();
        CombinedEncoder c = new CombinedEncoder(e1, 0.0, e2);
        e1.angle = 1; // this is the authority
        e2.angle = 0; // this is wrong
        // read the authority value
        assertEquals(0.0, c.getPositionRad().getAsDouble(), kDelta);
        // and fix the secondary
        assertEquals(0.0, e2.angle, kDelta);
    }
}
