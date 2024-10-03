package org.team100.lib.config;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Notice the difference between the naive feedforward and the more-correct
 * discretized one.
 */
class FeedforwardTest {
    private static final double kDelta = 0.001;

    @Test
    void testWPI() {
        SimpleMotorFeedforward smff = new SimpleMotorFeedforward(0, 1, 1);

        // current v = 1, want a = 1.
        // naive model says u = 2.
        assertEquals(2, smff.calculate(1, 1), kDelta);

        // current v = 1, a = 1 for 1 sec => next v = 2.
        // discrete model knows that u = 2 applied for 1 sec won't yield
        // v = 2 because the back EMF will increase along the way.
        // if you want v = 2 after 1 sec, you need to push harder.
        assertEquals(2.582, smff.calculate(1, 2, 1), kDelta);

        // for the usual time step this is a small correction.
        assertEquals(2.010, smff.calculate(1, 1.02, 0.02), kDelta);
    }

    @Test
    void test100() {
        // behaves the same as the naive model above, ignoring friction.
        Feedforward100 ff100 = new Feedforward100(1, 1, 0, 0, 0);
        assertEquals(1, ff100.velocityFFVolts(1), kDelta);
        assertEquals(1, ff100.accelFFVolts(1), kDelta);
    }

    /** I forgot an abs() in the friction term, so this verifies it. */
    @Test
    void testFriction() {
        // static friction = 2, dynamic friction = 1
        Feedforward100 ff100 = new Feedforward100(1, 1, 2, 1, 1);
        // under the static friction limit, so this is static
        assertEquals(2, ff100.frictionFFVolts(0.5, 0.5), kDelta);
        // over the static friction limit, so sliding
        assertEquals(1, ff100.frictionFFVolts(2, 2), kDelta);
        // under the static friction limit, so this is static
        assertEquals(-2, ff100.frictionFFVolts(-0.5, -0.5), kDelta);
        // over the static friction limit, so sliding
        assertEquals(-1, ff100.frictionFFVolts(-2, -2), kDelta);
        // moving positive, want to go negative, get negative
        assertEquals(-2, ff100.frictionFFVolts(0.5, -0.5), kDelta);
        // moving positive, want to go negative, get negative
        assertEquals(-1, ff100.frictionFFVolts(2, -2), kDelta);
    }

}
