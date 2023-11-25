package org.team100.lib.motion.crank;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.crank.Configuration;
import org.team100.lib.motion.crank.Kinematics;
import org.team100.lib.motion.crank.Workstate;

class CrankKinematicsTest {
    @Test
    void testForward() {
        Kinematics k = new Kinematics(1, 2);
        assertEquals(3, k.forward(new Configuration(0)).getState(), 0.001);
        assertEquals(Math.sqrt(3.5) + Math.sqrt(0.5),
                k.forward(new Configuration(Math.PI / 4)).getState(), 0.001);
        assertEquals(Math.sqrt(3), k.forward(new Configuration(Math.PI / 2)).getState(), 0.001);
        assertEquals(Math.sqrt(3.5) - Math.sqrt(0.5),
                k.forward(new Configuration(3 * Math.PI / 4)).getState(), 0.001);
        assertEquals(1, k.forward(new Configuration(Math.PI)).getState(), 0.001);
        assertEquals(Math.sqrt(3), k.forward(new Configuration(3 * Math.PI / 2)).getState(), 0.001);
    }

    @Test
    void testInverse() {
        Kinematics k = new Kinematics(1, 2);
        // same cases as above, note there are two crank cases for
        // each slider case; the inverse kinematics choose the positive one.
        assertEquals(0, k.inverse(new Workstate(3.0)).getCrankAngleRad(), 0.001);
        assertEquals(Math.PI / 4, k.inverse(new Workstate(Math.sqrt(3.5) + Math.sqrt(0.5))).getCrankAngleRad(),
                0.001);
        assertEquals(Math.PI / 2, k.inverse(new Workstate(Math.sqrt(3))).getCrankAngleRad(), 0.001);
        assertEquals(3 * Math.PI / 4, k.inverse(new Workstate(Math.sqrt(3.5) - Math.sqrt(0.5))).getCrankAngleRad(),
                0.001);
        assertEquals(Math.PI, k.inverse(new Workstate(1.0)).getCrankAngleRad(), 0.001);
    }
}
