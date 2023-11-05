package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Timer;

class MotionProfilePerformanceTest {
    @Test
    void testLotsOfThem() {
        MotionState start = new MotionState(0, 0);
        MotionState goal = new MotionState(1, 0);

        Timer timer = new Timer();
        timer.start();
        int count = 1000;
        for (int i = 0; i < count; ++i) {
            MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 1, 1, 1);
            assertNotNull(profile);
        }
        timer.stop();
        double timeS = timer.get();
        double timePerProfileMs = 1000 * timeS / count;
        // for slow machines this might be too low; it's ok to raise it a little
        // but if profiles are very slow that would be bad.
        assertTrue(timePerProfileMs < 0.2);
    }

}
