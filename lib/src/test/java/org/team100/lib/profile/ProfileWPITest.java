package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.State100;
import org.team100.lib.util.Util;

class ProfileWPITest {
    private static final double kDelta = 0.001;

    /**
     * this is a normal profile from 0 to 1, rest-to-rest, it's a triangle profile.
     */
    @Test
    void testTriangle() {
        ProfileWPI profileX = new ProfileWPI(5,2);
        State100 sample = new State100(0, 0);
        final State100 end = new State100(1, 0);

        double tt = 0;
        // the first sample is near the starting state
        Util.printf("%f %f %f\n", tt, sample.x(), sample.v());

        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.x(), sample.v());
        assertEquals(0, sample.x(), kDelta);
        assertEquals(0.04, sample.v(), kDelta);

        // step to the middle of the profile
        for (double t = 0; t < 0.68; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.x(), sample.v());
        }
        // halfway there, going fast
        assertEquals(0.5, sample.x(), 0.01);
        assertEquals(1.4, sample.v(), 0.01);

        // step to the end of the profile .. this was 0.72 before.
        for (double t = 0; t < 0.86; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.x(), sample.v());
        }
        assertEquals(1.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }
    
}
