package org.team100.lib.profile;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.trajectory.ExponentialProfile;

class ExponentialProfileTest {
    // just to look at the output
    @Test
    void testSimple() {
        ExponentialProfile.Constraints constraints = 
        ExponentialProfile.Constraints.fromCharacteristics(
            10, 1,
                1);
        ExponentialProfile eprofile = new ExponentialProfile(constraints);
        double position = 0;
        double velocity = 0;
        final double goalPosition = 1;
        final double goalVelocity = 0;
        for (double t = 0; t < 2; t += 0.02) {
            System.out.printf("%5.3f %5.3f %5.3f\n", t, position, velocity);
            ExponentialProfile.State s = eprofile.calculate(0.02,
                    new ExponentialProfile.State(position, velocity),
                    new ExponentialProfile.State(goalPosition, goalVelocity));
            position = s.position;
            velocity = s.velocity;
        }

    }

}
