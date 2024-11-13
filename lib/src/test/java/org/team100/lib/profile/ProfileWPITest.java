package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

class ProfileWPITest {
    private static final boolean actuallyPrint = false;
    private static final double kDelta = 0.001;

    private void dump(double tt, Control100 sample) {
        if (actuallyPrint)
            Util.printf("%f %f %f\n", tt, sample.x(), sample.v());
    }

    /**
     * this is a normal profile from 0 to 1, rest-to-rest, it's a triangle profile.
     */
    @Test
    void testTriangle() {
        ProfileWPI profileX = new ProfileWPI(5, 2);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(1, 0);

        double tt = 0;
        // the first sample is near the starting state
        dump(tt, sample);
        sample = profileX.calculate(0.02, sample.model(), end);
        tt += 0.02;
        dump(tt, sample);
        assertEquals(0, sample.x(), kDelta);
        assertEquals(0.04, sample.v(), kDelta);

        // step to the middle of the profile
        for (double t = 0; t < 0.68; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        // halfway there, going fast
        assertEquals(0.5, sample.x(), 0.01);
        assertEquals(1.4, sample.v(), 0.01);

        // step to the end of the profile .. this was 0.72 before.
        for (double t = 0; t < 0.86; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(1.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

}
