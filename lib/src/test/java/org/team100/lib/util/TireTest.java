package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.HashMap;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Vector2d;
import org.team100.lib.persistent_parameter.Parameter;
import org.team100.lib.persistent_parameter.ParameterFactory;

class TireTest {
    private static final double kDelta = 0.001;
    ParameterFactory factory = new ParameterFactory(new HashMap<>());
    Tire tire = new Tire(factory);

    @Test
    void testDesiredAccel() {
        // motionless
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
        Vector2d accel = new Vector2d(0.1, 0);
        assertEquals(0.1, tire.fraction(accel), kDelta);
        accel = new Vector2d(0.5, 0);
        assertEquals(0.5, tire.fraction(accel), kDelta);
        accel = new Vector2d(2.0, 0);
        assertEquals(2.0, tire.fraction(accel), kDelta);
    }

    @Test
    void testScale() {
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
        Vector2d scaledAccel = tire.scaledAccelM_s_s(new Vector2d(1.0, 0), 1.0);
        assertEquals(1.0, scaledAccel.getX(), kDelta);
        assertEquals(0, scaledAccel.getY(), kDelta);
        scaledAccel = tire.scaledAccelM_s_s(new Vector2d(1.0, 0), 0.5);
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
        Vector2d actual = tire.actual(new Vector2d(0, 0), new Vector2d(0, 0), 0.02);
        assertEquals(0, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testParallel() {
        // wheel wants to go the same speed as the corner
        Vector2d actual = tire.actual(new Vector2d(1, 0), new Vector2d(1, 0), 0.02);
        assertEquals(1, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testNear() {
        // wheel wants to speed up a little, slips a little
        Vector2d actual = tire.actual(new Vector2d(1, 0), new Vector2d(1.005, 0), 0.02);
        assertEquals(1.004875, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testMed() {
        // wheel wants to speed up a lot (0.5 m/s/s)
        Vector2d desired = tire.desiredAccelM_s_s(new Vector2d(1.0, 0), new Vector2d(1.01, 0), 0.02);
        assertEquals(0.5, desired.getX(), kDelta);
        assertEquals(0, desired.getY(), kDelta);
        // that's half the saturation value so it slips 5%
        Vector2d actual = tire.actual(new Vector2d(1, 0), new Vector2d(1.01, 0), 0.02);
        assertEquals(1.0095, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testSaturation() {
        // wheel is saturated, max dv is 0.02 * saturation (1) -> 0.02.
        Vector2d actual = tire.actual(new Vector2d(1, 0), new Vector2d(5.0, 0), 0.02);
        assertEquals(1.02, actual.getX(), kDelta);
        assertEquals(0, actual.getY(), kDelta);
    }

    @Test
    void testSaturation2() {
        // wheel is saturated in a different direction
        Vector2d actual = tire.actual(new Vector2d(1, 0), new Vector2d(0, 5.0), 0.02);
        assertEquals(0.996, actual.getX(), kDelta);
        assertEquals(0.020, actual.getY(), kDelta);
    }

    @Test
    void testNoSlipInfiniteSaturation() {
        ParameterFactory factory2 = new ParameterFactory(new HashMap<>());
        Tire tire2 = new Tire(factory2);
        Parameter saturation = factory2.mutable(Tire.kSaturationLabel, 0);
        // saturation.reset();
        Parameter slip = factory2.mutable(Tire.kSlipLabel, 0);
        // the defaults work
        assertEquals(1, saturation.get(), kDelta);
        assertEquals(0.1, slip.get(), kDelta);

        // override the defaults
        saturation.set(Double.MAX_VALUE);
        slip.set(0.0);
        // check that the overrides work
        assertEquals(Double.MAX_VALUE, saturation.get(), kDelta);
        assertEquals(0.0, slip.get(), kDelta);

        // try the saturated2 case.  here we want to stop the x motion and start some y motion,
        // all in 0.02s so these are high accelerations.
        Vector2d actual = tire2.actual(new Vector2d(1, 0), new Vector2d(0, 5.0), 0.02);
        // the tire sticks!
        assertEquals(0.0, actual.getX(), kDelta);
        assertEquals(5.0, actual.getY(), kDelta);
    }

    @Test
    void testApply() {
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
