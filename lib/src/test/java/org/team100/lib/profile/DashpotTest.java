package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.State100;

class DashpotTest {

    @Test
    void testFar() {
        MockProfile100 fast = new MockProfile100();
        MockProfile100 slow = new MockProfile100();
        Dashpot d = new Dashpot(fast, slow, 1, 1);
        d.calculate(1, new State100(10, 0, 0), new State100());
        assertEquals(1, fast.count);
        assertEquals(0, slow.count);
    }

    @Test
    void testNear() {
        MockProfile100 fast = new MockProfile100();
        MockProfile100 slow = new MockProfile100();
        Dashpot d = new Dashpot(fast, slow, 1, 1);
        d.calculate(1, new State100(0.5, 0, 0), new State100());
        assertEquals(0, fast.count);
        assertEquals(1, slow.count);
    }

    @Test
    void testSeries() {
        Profile100 fast = new TrapezoidProfile100(5, 5, 0.01);
        Profile100 slow = new TrapezoidProfile100(0.5, 5, 0.01);
        Dashpot d = new Dashpot(fast, slow, 0.2, 0.5);
        State100 setpoint = new State100();
        final State100 goal = new State100(1, 0);
        for (int i = 0; i < 100; ++i) {
            System.out.printf("%5.3f %5.3f %5.3f %5.3f\n", 0.02 * i, setpoint.x(), setpoint.v(), setpoint.a());
            setpoint = d.calculate(0.02, setpoint, goal);
        }
    }

    @Test
    void testSeries2() {
        Profile100 fast = new TrapezoidProfile100(5, 5, 0.01);
        Profile100 slow = new TrapezoidProfile100(0.5, 5, 0.01);
        Profile100 dd = new Dashpot(fast, slow, 0.2, 0.5);
        Profile100 d = new JerkLimiter(dd, 100);
        State100 setpoint = new State100();
        final State100 goal = new State100(1, 0);
        for (int i = 0; i < 100; ++i) {
            System.out.printf("%5.3f %5.3f %5.3f %5.3f\n", 0.02 * i, setpoint.x(), setpoint.v(), setpoint.a());
            setpoint = d.calculate(0.02, setpoint, goal);
        }
    }
}
