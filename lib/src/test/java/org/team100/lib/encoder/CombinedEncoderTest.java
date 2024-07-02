package org.team100.lib.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.units.Angle100;

class CombinedEncoderTest {
    private static final double kDelta = 0.001;

    @Test
    void testSimple1() {
        MockEncoder100<Angle100> e1 = new MockEncoder100<>();
        MockEncoder100<Angle100> e2 = new MockEncoder100<>();
        CombinedEncoder<Angle100> c = new CombinedEncoder<>(e1, 1.0, e2);
        e1.angle = 1; // this is the authority
        e2.angle = 0; // this is wrong
        // read the authority value
        assertEquals(1.0, c.getPosition().getAsDouble(), kDelta);
        // and fix the secondary
        assertEquals(1.0, e2.angle, kDelta);
    }

    @Test
    void testHalfPrimary() {
        MockEncoder100<Angle100> e1 = new MockEncoder100<>();
        MockEncoder100<Angle100> e2 = new MockEncoder100<>();
        CombinedEncoder<Angle100> c = new CombinedEncoder<>(e1, 0.5, e2);
        e1.angle = 1; // this is the authority
        e2.angle = 0; // this is wrong
        // read the authority value
        assertEquals(0.5, c.getPosition().getAsDouble(), kDelta);
        // and fix the secondary
        assertEquals(0.5, e2.angle, kDelta);
    }

    @Test
    void testIgnorePrimary() {
        MockEncoder100<Angle100> e1 = new MockEncoder100<>();
        MockEncoder100<Angle100> e2 = new MockEncoder100<>();
        CombinedEncoder<Angle100> c = new CombinedEncoder<>(e1, 0.0, e2);
        e1.angle = 1; // this is the authority
        e2.angle = 0; // this is wrong
        // read the authority value
        assertEquals(0.0, c.getPosition().getAsDouble(), kDelta);
        // and fix the secondary
        assertEquals(0.0, e2.angle, kDelta);
    }
}
