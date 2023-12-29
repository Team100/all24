package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;


class TrapezoidProfile100Test {

    boolean dump = false;
    private static final double kDelta = 0.001;

    @Test
    void testProfile() {
        TrapezoidProfile100.Constraints c = new TrapezoidProfile100.Constraints(5, 2);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c);
        TrapezoidProfile100.State sample = new TrapezoidProfile100.State(0, 0);
        final TrapezoidProfile100.State end = new TrapezoidProfile100.State(1, 0);
        for (double t = 0; t < 2; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            if (dump)
                Util.printf("%f %f %f\n", t, sample.position, sample.velocity);
            if (profileX.totalTime() < 0.001)
                break;
        }
        // no time left
        assertEquals(0, profileX.totalTime(), kDelta);
    }

}
