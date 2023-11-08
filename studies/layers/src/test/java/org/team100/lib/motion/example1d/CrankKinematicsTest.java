package org.team100.lib.motion.example1d;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class CrankKinematicsTest {
    @Test
    void testForward() {
        Kinematics1d k = new CrankKinematics(1,2);
        assertEquals(3, k.forward(0), 0.001);
        assertEquals(Math.sqrt(3.5) + Math.sqrt(0.5), k.forward(Math.PI/4), 0.001);
        assertEquals(Math.sqrt(3), k.forward(Math.PI/2), 0.001);
        assertEquals(Math.sqrt(3.5) - Math.sqrt(0.5), k.forward(3*Math.PI/4), 0.001);
        assertEquals(1, k.forward(Math.PI), 0.001);
        assertEquals(Math.sqrt(3), k.forward(3*Math.PI/2), 0.001);
    }

    @Test
    void testInverse() {
        Kinematics1d k = new CrankKinematics(1,2);
        // same cases as above, note there are two crank cases for
        // each slider case; the inverse kinematics choose the positive one.
        assertEquals(0, k.inverse(3), 0.001);
        assertEquals(Math.PI/4, k.inverse(Math.sqrt(3.5) + Math.sqrt(0.5)), 0.001);
        assertEquals(Math.PI/2, k.inverse(Math.sqrt(3)), 0.001);
        assertEquals(3*Math.PI/4, k.inverse(Math.sqrt(3.5) - Math.sqrt(0.5)), 0.001);
        assertEquals(Math.PI, k.inverse(1), 0.001);
    }
}
