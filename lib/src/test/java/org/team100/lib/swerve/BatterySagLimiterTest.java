package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class BatterySagLimiterTest {
    private static final double kDelta = 0.001;

    double volts;

    @Test
    void testSimple() {
        BatterySagLimiter l = new BatterySagLimiter(() -> volts);
        volts = 0;
        assertEquals(0.00, l.get(), kDelta);
        volts = 6;
        assertEquals(0.00, l.get(), kDelta);
        volts = 6.25;
        assertEquals(0.25, l.get(), kDelta);
        volts = 6.5;
        assertEquals(0.50, l.get(), kDelta);
        volts = 6.75;
        assertEquals(0.75, l.get(), kDelta);
        volts = 7;
        assertEquals(1.00, l.get(), kDelta);
        volts = 8;
        assertEquals(1.00, l.get(), kDelta);
        volts = 12;
        assertEquals(1.00, l.get(), kDelta);
    }

}
