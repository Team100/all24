package org.team100.lib.controller.simple;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.State100;

import edu.wpi.first.math.MathUtil;

class FullStateControllerTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testZero() {
        FullStateController c = new FullStateController(logger, 4, 0.25, x -> x, 0.01, 0.01);
        double u = c.calculate(new State100(0, 0), new State100(0, 0));
        assertEquals(0, u, kDelta);
    }

    @Test
    void testK1() {
        FullStateController c = new FullStateController(logger, 4, 0.25, x -> x, 0.01, 0.01);
        double u = c.calculate(new State100(0, 0), new State100(1, 0));
        assertEquals(4, u, kDelta);
    }

    @Test
    void testK1b() {
        FullStateController c = new FullStateController(logger, 4, 0.25, x -> x, 0.01, 0.01);
        double u = c.calculate(new State100(1, 0), new State100(0, 0));
        assertEquals(-4, u, kDelta);
    }

    @Test
    void testKangle() {
        FullStateController c = new FullStateController(logger, 4, 0.25, MathUtil::angleModulus, 0.01, 0.01);
        // at -3, near pi, goal is 3, across pi
        double u = c.calculate(new State100(-3, 0), new State100(3, 0));
        // the correct course is reverse
        assertEquals(-1.133, u, kDelta);
    }

    @Test
    void testK2() {
        FullStateController c = new FullStateController(logger, 4, 0.25, x -> x, 0.01, 0.01);
        double u = c.calculate(new State100(0, 0), new State100(0, 1));
        // feedforward = reference velocity
        // feedback = velocity error * k2
        assertEquals(1.25, u, kDelta);
    }

    @Test
    void testK2b() {
        FullStateController c = new FullStateController(logger, 4, 0.25, x -> x, 0.01, 0.01);
        double u = c.calculate(new State100(0, 1), new State100(0, 0));
        // slow down
        assertEquals(-0.25, u, kDelta);
    }
}
