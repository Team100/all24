package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class BatterySagLimiterTest {
    private static final double kDelta = 0.001;

    @Test
    void testSimple() {
        BatterySagLimiter l = new BatterySagLimiter();
        assertEquals(0.00, l.get(0), kDelta);
        assertEquals(0.00, l.get(6), kDelta);
        assertEquals(0.25, l.get(6.25), kDelta);
        assertEquals(0.50, l.get(6.5), kDelta);
        assertEquals(0.75, l.get(6.75), kDelta);
        assertEquals(1.00, l.get(7), kDelta);
        assertEquals(1.00, l.get(8), kDelta);
        assertEquals(1.00, l.get(12), kDelta);

    }

}
