package org.team100.lib.units;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class DistanceTest {
    private static final double kDelta = 0.001;

    /** Modulus is identity */
    @Test
    void testModulus() {
        Measure100 m = Distance.instance;
        assertEquals(0, m.modulus(0), kDelta);
        assertEquals(1, m.modulus(1), kDelta);
        assertEquals(2, m.modulus(2), kDelta);
    }
}
