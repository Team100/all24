package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

class TrapezoidProfileTest {
    private static final double kDelta = 0.001;

    @Test
    void testProfile() {
        TrapezoidProfile.Constraints c = new TrapezoidProfile.Constraints(5, 2);
        TrapezoidProfile profileX = new TrapezoidProfile(c);
        TrapezoidProfile.State sample = new TrapezoidProfile.State(0, 0);
        final TrapezoidProfile.State end = new TrapezoidProfile.State(1, 0);
        for (double t = 0; t < 2; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            System.out.printf("%f %f %f\n", t, sample.position, sample.velocity);
            if (profileX.totalTime() < 0.001)
                break;
        }
        // no time left
        assertEquals(0, profileX.totalTime(), kDelta);
    }

}
