package org.team100.lib.motion.crank;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.crank.CrankConfiguration;
import org.team100.lib.motion.crank.CrankKinematics;
import org.team100.lib.motion.crank.CrankWorkstate;

class CrankKinematicsTest {
    @Test
    void testForward() {
        CrankKinematics k = new CrankKinematics(1, 2);
        assertEquals(3, k.forward(new CrankConfiguration(0)).getState(), 0.001);
        assertEquals(Math.sqrt(3.5) + Math.sqrt(0.5),
                k.forward(new CrankConfiguration(Math.PI / 4)).getState(), 0.001);
        assertEquals(Math.sqrt(3), k.forward(new CrankConfiguration(Math.PI / 2)).getState(), 0.001);
        assertEquals(Math.sqrt(3.5) - Math.sqrt(0.5),
                k.forward(new CrankConfiguration(3 * Math.PI / 4)).getState(), 0.001);
        assertEquals(1, k.forward(new CrankConfiguration(Math.PI)).getState(), 0.001);
        assertEquals(Math.sqrt(3), k.forward(new CrankConfiguration(3 * Math.PI / 2)).getState(), 0.001);
    }

    @Test
    void testInverse() {
        CrankKinematics k = new CrankKinematics(1, 2);
        // same cases as above, note there are two crank cases for
        // each slider case; the inverse kinematics choose the positive one.
        assertEquals(0, k.inverse(new CrankWorkstate(3.0)).getCrankAngleRad(), 0.001);
        assertEquals(Math.PI / 4, k.inverse(new CrankWorkstate(Math.sqrt(3.5) + Math.sqrt(0.5))).getCrankAngleRad(),
                0.001);
        assertEquals(Math.PI / 2, k.inverse(new CrankWorkstate(Math.sqrt(3))).getCrankAngleRad(), 0.001);
        assertEquals(3 * Math.PI / 4, k.inverse(new CrankWorkstate(Math.sqrt(3.5) - Math.sqrt(0.5))).getCrankAngleRad(),
                0.001);
        assertEquals(Math.PI, k.inverse(new CrankWorkstate(1.0)).getCrankAngleRad(), 0.001);
    }
}
