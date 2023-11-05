package org.team100.lib.kinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmAngles;

import edu.wpi.first.math.geometry.Translation2d;

class ArmKinematicsTest {
    private static final double kDelta = 0.001;

    @Test
    void testForward1() {
        ArmKinematics k = new ArmKinematics(0.93, 0.92);// like 2023
        ArmAngles a = new ArmAngles(0, Math.PI / 2);
        Translation2d t = k.forward(a);
        assertEquals(0.93, t.getX(), kDelta); // lower arm length
        assertEquals(0.92, t.getY(), kDelta); // upper arm length
    }

    @Test
    void testInverse1() {
        ArmKinematics k = new ArmKinematics(0.93, 0.92);// like 2023
        Translation2d t = new Translation2d(0.93, 0.92);
        ArmAngles a = k.inverse(t);
        assertEquals(0, a.th1, kDelta);
        assertEquals(Math.PI / 2, a.th2, kDelta);
    }

    @Test
    void testForward2() {
        ArmKinematics k = new ArmKinematics(0.93, 0.92);// like 2023
        ArmAngles a = new ArmAngles(Math.PI / 4, Math.PI / 2);
        Translation2d t = k.forward(a);
        assertEquals(0.658, t.getX(), kDelta); // 0.93 * sqrt(2)/2
        assertEquals(1.578, t.getY(), kDelta); // 0.92 + 0.93 * sqrt(2)/2
    }

    @Test
    void testInverse2() {
        ArmKinematics k = new ArmKinematics(0.93, 0.92);// like 2023
        Translation2d t = new Translation2d(0.658, 1.578);
        ArmAngles a = k.inverse(t);
        assertEquals(Math.PI / 4, a.th1, kDelta);
        assertEquals(Math.PI / 2, a.th2, kDelta);
    }

    @Test
    void testForward3() {
        ArmKinematics k = new ArmKinematics(0.93, 0.92); // like 2023
        ArmAngles a = new ArmAngles(Math.PI / 4, Math.PI / 4);
        Translation2d t = k.forward(a);
        assertEquals(1.308, t.getX(), kDelta); // (0.92 + 0.93) * sqrt(2)/2
        assertEquals(1.308, t.getY(), kDelta); // (0.92 + 0.93) * sqrt(2)/2
    }

    @Test
    void testInverse3() {
        ArmKinematics k = new ArmKinematics(0.93, 0.92);// like 2023
        Translation2d t = new Translation2d(1.308147, 1.308147);
        ArmAngles a = k.inverse(t);
        // the angles here are very sensitive because the elbow is fully extended.
        assertEquals(Math.PI / 4, a.th1, kDelta);
        assertEquals(Math.PI / 4, a.th2, kDelta);
    }

    @Test
    void testForward4() {
        ArmKinematics k = new ArmKinematics(1, 1); // unit length
        ArmAngles a = new ArmAngles(0, Math.PI);
        Translation2d t = k.forward(a);
        assertEquals(0, t.getX(), kDelta);
        assertEquals(0, t.getY(), kDelta);
    }

    @Test
    void testInverse4() {
        ArmKinematics k = new ArmKinematics(1, 1); // unit length
        Translation2d t = new Translation2d(0, 0);
        ArmAngles a = k.inverse(t);
        assertNull(a); // this case is underdetermined
    }

    @Test
    void testForward5() {
        ArmKinematics k = new ArmKinematics(1, 1); // unit length
        ArmAngles a = new ArmAngles(Math.PI / 4, 3 * Math.PI / 4);
        Translation2d t = k.forward(a);
        assertEquals(0, t.getX(), kDelta);
        assertEquals(1.414, t.getY(), kDelta); // sqrt(2)
    }

    @Test
    void testInverse5() {
        ArmKinematics k = new ArmKinematics(1, 1);// unit length
        Translation2d t = new Translation2d(0, 1.414);// sqrt(2)
        ArmAngles a = k.inverse(t);
        assertEquals(Math.PI / 4, a.th1, kDelta);
        assertEquals(3 * Math.PI / 4, a.th2, kDelta);
    }

    @Test
    void testInverseUnreachable() {
        ArmKinematics k = new ArmKinematics(1, 1); // unit length
        Translation2d t = new Translation2d(2, 2);
        ArmAngles a = k.inverse(t);
        assertNull(a); // this case is unreachable
    }
}
