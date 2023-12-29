package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

class TrapezoidProfile100Test {

    boolean dump = true;
    private static final double kDelta = 0.001;

    @Test
    void testProfile() {
        Constraints c = new Constraints(5, 2);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c);
        State sample = new State(0, 0);
        final State end = new State(1, 0);
        for (double t = 0; t < 2; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);

            Util.printf("%f %f %f\n", t, sample.position, sample.velocity);
            if (profileX.totalTime() < 0.001)
                break;
        }
        // no time left
        assertEquals(0, profileX.totalTime(), kDelta);
    }

    // heading away from the goal, overshoot works correctly.
    @Test
    void testProfile2() {
        Constraints c = new Constraints(5, 1);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c);
        State sample = new State(0.1, 1);
        final State end = new State(0, 0);
        for (double t = 0; t < 10; t += 0.02) {
            Util.printf("%f %f %f\n", t, sample.position, sample.velocity);
            sample = profileX.calculate(0.02, end, sample);

            if (profileX.totalTime() < 0.001)
                break;
        }
        // no time left
        assertEquals(0, profileX.totalTime(), kDelta);
    }

    // heading too fast towards the goal, it jumps the position back at infinite
    // speed and jumps the velocity down at infinite acceleration, to end at the
    // goal.
    @Test
    void testProfile3() {
        Constraints c = new Constraints(5, 1);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c);
        State sample = new State(0.1, 1);
        final State end = new State(0, 0);
        for (double t = 0; t < 10; t += 0.02) {
            Util.printf("%f %f %f\n", t, sample.position, sample.velocity);
            sample = profileX.calculate(0.02, end, sample);

            if (profileX.totalTime() < 0.001)
                break;
        }
        // no time left
        assertEquals(0, profileX.totalTime(), kDelta);
    }
}
