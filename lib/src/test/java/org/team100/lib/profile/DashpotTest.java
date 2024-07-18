package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.State100;

class DashpotTest {

    @Test
    void testFar() {
        MockProfile100 fast = new MockProfile100();
        MockProfile100 slow = new MockProfile100();
        Dashpot d = new Dashpot(fast, slow, 1);
        d.calculate(1, new State100(10, 0, 0), new State100());
        assertEquals(1, fast.count);
        assertEquals(0, slow.count);
    }
    @Test
    void testNear() {
        MockProfile100 fast = new MockProfile100();
        MockProfile100 slow = new MockProfile100();
        Dashpot d = new Dashpot(fast, slow, 1);
        d.calculate(1, new State100(0.5, 0, 0), new State100());
        assertEquals(0, fast.count);
        assertEquals(1, slow.count);
    }
}
