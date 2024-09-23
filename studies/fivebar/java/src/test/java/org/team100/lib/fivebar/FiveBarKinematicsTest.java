package org.team100.lib.fivebar;

import static org.junit.Assert.assertEquals;

import org.junit.jupiter.api.Test;
import static java.lang.Math.sqrt;

class FiveBarKinematicsTest {
    private static final double kDelta = 0.001;

    private Scenario regularPentagon() {
        Scenario s = new Scenario();
        // unit side length
        // all sides the same
        s.a1 = 1.0;
        s.a2 = 1.0;
        s.a3 = 1.0;
        s.a4 = 1.0;
        s.a5 = 1.0;
        return s;
    }

    /**
     * A regular pentagon
     * https://en.wikipedia.org/wiki/Pentagon
     */
    @Test
    void testPentagonInverse() {
        Scenario s = regularPentagon();
        // endpoint x in the center of a5
        double x3 = -0.5 * 1.0;
        // height
        double y3 = 1.0 * sqrt(5 + 2 * sqrt(5)) / 2;
        // check the height
        assertEquals(1.539, y3, kDelta);
        ActuatorAngles p = FiveBarKinematics.inverse(s, x3, y3);
        // 360/5 = 72 degrees
        assertEquals(1.256, p.thetaOne, kDelta);
        // the complement
        assertEquals(1.885, p.thetaFive, kDelta);
    }

    /** the inverse of the case above */
    @Test
    void testPentagonForward() {
        Scenario s = regularPentagon();
        double t1 = 1.256;
        double t5 = 1.885;
        JointPositions j = FiveBarKinematics.forward(s, t1, t5);
        assertEquals(-0.5, j.P3.x, kDelta);
        assertEquals(1.539, j.P3.y, kDelta);
    }

    private Scenario littleHouse() {
        Scenario s = new Scenario();
        // unit side length
        s.a1 = 1.0;
        s.a2 = 0.5 * sqrt(2);
        s.a3 = 0.5 * sqrt(2);
        s.a4 = 1.0;
        s.a5 = 1.0;
        return s;
    }

    /** A square with a roof */
    @Test
    void testLittleHouseInverse() {
        Scenario s = littleHouse();
        // endpoint x in the center of a5
        double x3 = -0.5 ;
        // height
        double y3 =  1.5;
        // check the height
        ActuatorAngles p = FiveBarKinematics.inverse(s, x3, y3);
        // 90 degrees
        assertEquals(1.571, p.thetaOne, kDelta);
        // also 90 degrees
        assertEquals(1.571, p.thetaFive, kDelta);
    }

    /** Inverse of the case above */
    @Test
    void testLittleHouseForward() {
        Scenario s = littleHouse();
        double t1 = 1.571;
        double t5 = 1.571;
        JointPositions j = FiveBarKinematics.forward(s, t1, t5);
        assertEquals(-0.5, j.P3.x, kDelta);
        assertEquals(1.5, j.P3.y, kDelta);
    }
}
