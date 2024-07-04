package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Vector2d;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Logger;

class TireTest {
    private static final double kDelta = 0.001;
    Logger logger = Telemetry.get().testLogger();

    @Test
    void testDesiredAccel() {
        // motionless
        Tire tire = new Tire(logger, 10, 0.1);
        Vector2d actual = tire.desiredAccelM_s_s(new Vector2d(0, 0), new Vector2d(0, 0), 0.02);
        assertEquals(0, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
        // wheel wants to go the same speed as the corner
        actual = tire.desiredAccelM_s_s(new Vector2d(1.0, 0), new Vector2d(1.0, 0), 0.02);
        assertEquals(0, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
        // wheel wants to speed up, 0.01 m/s in 0.02 s => 0.5 m/s/s
        actual = tire.desiredAccelM_s_s(new Vector2d(1.0, 0), new Vector2d(1.01, 0), 0.02);
        assertEquals(0.5, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
        // wheel wants to speed up, 0.1 m/s in 0.02 s => 5 m/s/s
        actual = tire.desiredAccelM_s_s(new Vector2d(1.0, 0), new Vector2d(1.1, 0), 0.02);
        assertEquals(5.0, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testFraction() {
        Vector2d accel = new Vector2d(1, 0);
        Tire tire = new Tire(logger, 10, 0.1);
        assertEquals(0.1, tire.fraction(accel), kDelta);
        accel = new Vector2d(5, 0);
        assertEquals(0.5, tire.fraction(accel), kDelta);
        accel = new Vector2d(20, 0);
        assertEquals(2.0, tire.fraction(accel), kDelta);
    }

    @Test
    void testScale() {
        Tire tire = new Tire(logger, 10, 0.1);
        assertEquals(1.000, tire.scale(-1.00), kDelta);
        assertEquals(1.000, tire.scale(0.00), kDelta);
        assertEquals(0.975, tire.scale(0.25), kDelta);
        assertEquals(0.950, tire.scale(0.50), kDelta);
        assertEquals(0.925, tire.scale(0.75), kDelta);
        assertEquals(0.900, tire.scale(1.00), kDelta);
        assertEquals(0.900, tire.scale(1.25), kDelta);
        assertEquals(0.900, tire.scale(1.50), kDelta);
        assertEquals(0.900, tire.scale(1.75), kDelta);
        assertEquals(0.900, tire.scale(2.0), kDelta);
    }

    @Test
    void testScaledAccel() {
        Tire tire = new Tire(logger, 10, 0.1);
        Vector2d scaledAccel = tire.scaledAccelM_s_s(new Vector2d(1.0, 0), 1.0);
        assertEquals(1.0, scaledAccel.getX(), kDelta);
        assertEquals(0, scaledAccel.getY(), kDelta);
        scaledAccel = tire.scaledAccelM_s_s(new Vector2d(1.0, 0), 0.5);
        assertEquals(0.5, scaledAccel.getX(), kDelta);
        assertEquals(0, scaledAccel.getY(), kDelta);
    }

    @Test
    void testLimit() {
        Tire tire = new Tire(logger, 10, 0.1);
        Vector2d limitedAccel = tire.limit(new Vector2d(10, 0));
        assertEquals(10, limitedAccel.getX(), kDelta);
        assertEquals(0, limitedAccel.getY(), kDelta);

        limitedAccel = tire.limit(new Vector2d(15, 0));
        assertEquals(10, limitedAccel.getX(), kDelta);
        assertEquals(0, limitedAccel.getY(), kDelta);

        limitedAccel = tire.limit(new Vector2d(5, 0));
        assertEquals(5, limitedAccel.getX(), kDelta);
        assertEquals(0, limitedAccel.getY(), kDelta);

        limitedAccel = tire.limit(new Vector2d(-5, 0));
        assertEquals(-5, limitedAccel.getX(), kDelta);
        assertEquals(0, limitedAccel.getY(), kDelta);

        limitedAccel = tire.limit(new Vector2d(-15, 0));
        assertEquals(-10, limitedAccel.getX(), kDelta);
        assertEquals(0, limitedAccel.getY(), kDelta);
    }

    @Test
    void testMotionless() {
        Tire tire = new Tire(logger, 10, 0.1);
        Vector2d actual = tire.actual(
                new Vector2d(0, 0), new Vector2d(0, 0), 0.02);
        assertEquals(0, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testParallel() {
        // wheel wants to go the same speed as the corner
        Tire tire = new Tire(logger, 10, 0.1);
        Vector2d actual = tire.actual(
                new Vector2d(1, 0), new Vector2d(1, 0), 0.02);
        assertEquals(1, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testNear() {
        // wheel wants to speed up a little, slips a little
        Tire tire = new Tire(logger, 10, 0.1);
        Vector2d actual = tire.actual(
                new Vector2d(1, 0), new Vector2d(1.005, 0), 0.02);
        assertEquals(1.004875, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testMed() {
        // wheel wants to speed up a lot (0.5 m/s/s)
        Tire tire = new Tire(logger, 10, 0.1);
        Vector2d desired = tire.desiredAccelM_s_s(new Vector2d(1.0, 0), new Vector2d(1.01, 0), 0.02);
        assertEquals(0.5, desired.getX(), kDelta);
        assertEquals(0, desired.getY(), kDelta);
        // that's half the saturation value so it slips 5%
        Vector2d actual = tire.actual(
                new Vector2d(1, 0), new Vector2d(1.01, 0), 0.02);
        assertEquals(1.0095, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testSaturation() {
        // wheel is saturated, max dv is 0.02 * saturation (10) -> 0.2.
        Tire tire = new Tire(logger, 10, 0.1);
        Vector2d actual = tire.actual(
                new Vector2d(1, 0), new Vector2d(5.0, 0), 0.02);
        assertEquals(1.2, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testSaturation2() {
        // wheel is saturated in a different direction
        Tire tire = new Tire(logger, 10, 0.1);
        Vector2d actual = tire.actual(
                new Vector2d(1, 0), new Vector2d(0, 5.0), 0.02);
        assertEquals(0.961, actual.getX(), kDelta);
        assertEquals(0.196, actual.getY(), kDelta);
    }

    @Test
    void testNoSlipInfiniteSaturation() {
        // try the saturated2 case. here we want to stop the x motion and start some y
        // motion, all in 0.02s so these are high accelerations.
        Tire tire2 = new Tire(logger, Double.MAX_VALUE, 0.0);
        Vector2d actual = tire2.actual(new Vector2d(1, 0), new Vector2d(0, 5.0), 0.02);
        // the tire sticks!
        assertEquals(0.0, actual.getX(), kDelta);
        assertEquals(5.0, actual.getY(), kDelta);
    }

    @Test
    void testApply() {
        Tire tire = new Tire(logger, 10, 0.1);
        Vector2d result = tire.apply(new Vector2d(0, 0), new Vector2d(0, 0), 0.02);
        assertEquals(0, result.getX(), kDelta);
        assertEquals(0, result.getY(), kDelta);
        result = tire.apply(new Vector2d(0, 0), new Vector2d(1, 0), 0.02);
        assertEquals(0.02, result.getX(), kDelta);
        assertEquals(0, result.getY(), kDelta);
        result = tire.apply(new Vector2d(1, 0), new Vector2d(1, 0), 0.02);
        assertEquals(1.02, result.getX(), kDelta);
        assertEquals(0, result.getY(), kDelta);
    }
}
