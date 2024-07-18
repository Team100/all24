package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.State100;

class JerkLimiterTest {
    private static final double kDelta = 0.001;

    @Test
    void testOK() {
        MockProfile100 m = new MockProfile100();
        m.result = new State100(0, 0, 0.01);
        JerkLimiter jl = new JerkLimiter(m, 1);
        State100 r = jl.calculate(0.02, new State100(), new State100());
        assertEquals(0, r.x(), kDelta);
        assertEquals(0, r.v(), kDelta);
        assertEquals(0.01, r.a(), kDelta);
    }

    @Test
    void testCorrected() {
        MockProfile100 m = new MockProfile100();
        m.result = new State100(0, 0, 2);
        JerkLimiter jl = new JerkLimiter(m, 1);
        State100 r = jl.calculate(0.02, new State100(), new State100());
        assertEquals(0, r.x(), kDelta);
        assertEquals(0, r.v(), kDelta);
        assertEquals(0.02, r.a(), kDelta);
    }
}
