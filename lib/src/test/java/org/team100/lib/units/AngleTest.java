package org.team100.lib.units;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class AngleTest {
    private static final double kDelta = 0.001;

    /** Modulus is periodic. */
    @Test
    void testModulus() {
        Measure m = Angle.instance;
        assertEquals(0, m.modulus(0), kDelta);
        assertEquals(Math.PI, m.modulus(Math.PI), kDelta);
        assertEquals(0, m.modulus(2 * Math.PI), kDelta);
        assertEquals(Math.PI, m.modulus(3 * Math.PI), kDelta);

    }
}
