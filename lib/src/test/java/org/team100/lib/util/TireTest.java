package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.HashMap;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Vector2d;
import org.team100.lib.persistent_parameter.ParameterFactory;

class TireTest {
    private static final double kDelta = 0.001;
    ParameterFactory factory = new ParameterFactory(new HashMap<>());
    Tire tire = new Tire(factory);

    @Test
    void testDesiredAccel() {
        // motionless
        Vector2d actual = tire.desiredAccel(new Vector2d(0, 0), new Vector2d(0, 0));
        assertEquals(0, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
        // wheel wants to go the same speed as the corner
        actual = tire.desiredAccel(new Vector2d(1.0, 0), new Vector2d(1.0, 0));
        assertEquals(0, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
        // wheel wants to speed up
        actual = tire.desiredAccel(new Vector2d(1.0, 0), new Vector2d(1.1, 0));
        assertEquals(0.1, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testFraction() {
        Vector2d accel = new Vector2d(0.1, 0);
        assertEquals(0.1, tire.fraction(accel), kDelta);
        accel = new Vector2d(0.5, 0);
        assertEquals(0.5, tire.fraction(accel), kDelta);
        accel = new Vector2d(2.0, 0);
        assertEquals(2.0, tire.fraction(accel), kDelta);
    }

    @Test
    void testScale() {
        assertEquals(1.000, tire.scale(0.00), kDelta);
        assertEquals(0.975, tire.scale(0.25), kDelta);
        assertEquals(0.950, tire.scale(0.50), kDelta);
        assertEquals(0.925, tire.scale(0.75), kDelta);
        assertEquals(0.900, tire.scale(1.00), kDelta);
        assertEquals(0.875, tire.scale(1.25), kDelta);
        assertEquals(0.850, tire.scale(1.50), kDelta);
        assertEquals(0.825, tire.scale(1.75), kDelta);
        assertEquals(0.800, tire.scale(2.0), kDelta);
    }

    @Test
    void testScaledAccel() {
        Vector2d scaledAccel = tire.scaledAccel(new Vector2d(1.0, 0), 1.0);
        assertEquals(1.0, scaledAccel.getX(), kDelta);
        assertEquals(0, scaledAccel.getY(), kDelta);
        scaledAccel = tire.scaledAccel(new Vector2d(1.0, 0), 0.5);
        assertEquals(0.5, scaledAccel.getX(), kDelta);
        assertEquals(0, scaledAccel.getY(), kDelta);
    }

    @Test
    void testLimit() {
        Vector2d limitedAccel = tire.limit(new Vector2d(1.0, 0));
        assertEquals(1.0, limitedAccel.getX(), kDelta);
        assertEquals(0, limitedAccel.getY(), kDelta);
        limitedAccel = tire.limit(new Vector2d(1.5, 0));
        assertEquals(1.0, limitedAccel.getX(), kDelta);
        assertEquals(0, limitedAccel.getY(), kDelta);
        limitedAccel = tire.limit(new Vector2d(0.5, 0));
        assertEquals(0.5, limitedAccel.getX(), kDelta);
        assertEquals(0, limitedAccel.getY(), kDelta);
        limitedAccel = tire.limit(new Vector2d(-0.5, 0));
        assertEquals(-0.5, limitedAccel.getX(), kDelta);
        assertEquals(0, limitedAccel.getY(), kDelta);
        limitedAccel = tire.limit(new Vector2d(-1.5, 0));
        assertEquals(-1.0, limitedAccel.getX(), kDelta);
        assertEquals(0, limitedAccel.getY(), kDelta);
    }

    @Test
    void testMotionless() {
        Vector2d actual = tire.actual(new Vector2d(0, 0), new Vector2d(0, 0));
        assertEquals(0, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testParallel() {
        // wheel wants to go the same speed as the corner
        Vector2d actual = tire.actual(new Vector2d(1, 0), new Vector2d(1, 0));
        assertEquals(1, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testNear() {
        // wheel wants to speed up a little, slips a little
        Vector2d actual = tire.actual(new Vector2d(1, 0), new Vector2d(1.1, 0));
        assertEquals(1.099, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testMed() {
        // wheel wants to speed up a lot, slips a lot
        Vector2d actual = tire.actual(new Vector2d(1, 0), new Vector2d(2.0, 0));
        assertEquals(1.9, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testSaturation() {
        // wheel is saturated
        Vector2d actual = tire.actual(new Vector2d(1, 0), new Vector2d(5.0, 0));
        assertEquals(2.0, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testSaturation2() {
        // wheel is saturated in a different direction
        Vector2d actual = tire.actual(new Vector2d(1, 0), new Vector2d(0, 5.0));
        assertEquals(0.804, actual.getX(), kDelta);
        assertEquals(0.981, actual.getY(), kDelta);
    }
}
