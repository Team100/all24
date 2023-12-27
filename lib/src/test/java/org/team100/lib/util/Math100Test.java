package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.function.DoubleBinaryOperator;

import org.junit.jupiter.api.Test;

class Math100Test {
    private static final double kDelta = 0.001;

    @Test
    void testEpsilonEquals() {
        assertTrue(Math100.epsilonEquals(1, 1));
        assertFalse(Math100.epsilonEquals(1, 1.01));
    }

    @Test
    void testSolveQuadraticTwo() {
        List<Double> roots = Math100.solveQuadratic(1, 0, -1);
        assertEquals(2, roots.size());
        assertEquals(1, roots.get(0), kDelta);
        assertEquals(-1, roots.get(1), kDelta);
    }

    @Test
    void testSolveQuadraticOne() {
        List<Double> roots = Math100.solveQuadratic(1, 2, 1);
        assertEquals(1, roots.size());
        assertEquals(-1, roots.get(0), kDelta);
    }

    @Test
    void testSolveQuadraticZero() {
        List<Double> roots = Math100.solveQuadratic(1, 0, 1);
        assertEquals(0, roots.size());
    }

    @Test
    void testFindRootLinear() {

        double x_0 = -1;
        double y_0 = 0;
        double f_0 = -1;
        double x_1 = 1;
        double y_1 = 0;
        double f_1 = 1;
        int max_iterations = 1000;

        DoubleBinaryOperator func = (x, y) -> {
            return x;
        };

        // make sure we're passing the right bounds
        assertEquals(f_0, func.applyAsDouble(x_0, y_0), kDelta);
        assertEquals(f_1, func.applyAsDouble(x_1, y_1), kDelta);

        double s = Math100.findRoot(func, x_0, y_0, f_0, x_1, y_1, f_1, max_iterations);

        // between -1 and 1 the zero is half way
        assertEquals(0.5, s, kDelta);
    }

    @Test
    void testFindRootLinear2() {

        double x_0 = -1;
        double y_0 = 0;
        double f_0 = -1;
        double x_1 = 0.5;
        double y_1 = 0;
        double f_1 = 0.5;
        int max_iterations = 1000;

        DoubleBinaryOperator func = (x, y) -> {
            return x;
        };

        // make sure we're passing the right bounds
        assertEquals(f_0, func.applyAsDouble(x_0, y_0), kDelta);
        assertEquals(f_1, func.applyAsDouble(x_1, y_1), kDelta);

        double s = Math100.findRoot(func, x_0, y_0, f_0, x_1, y_1, f_1, max_iterations);

        // between -1 and 0.5 the zero is at s = 2/3
        assertEquals(0.667, s, kDelta);
    }

    @Test
    void testFindRootLinear2d() {

        double x_0 = -1;
        double y_0 = -1;
        double f_0 = -2;
        double x_1 = 1;
        double y_1 = 1;
        double f_1 = 2;
        int max_iterations = 1000;

        DoubleBinaryOperator func = (x, y) -> {
            return x + y;
        };

        // make sure we're passing the right bounds
        assertEquals(f_0, func.applyAsDouble(x_0, y_0), kDelta);
        assertEquals(f_1, func.applyAsDouble(x_1, y_1), kDelta);

        double s = Math100.findRoot(func, x_0, y_0, f_0, x_1, y_1, f_1, max_iterations);

        // the "zero" of x+y is a line, but this method only looks at
        // the line between x0,y0 and x1,y1, so it find the intersection at 0.
        assertEquals(0.5, s, kDelta);
    }

    @Test
    void testFindRootLog() {

        double x_0 = 0.1;
        double y_0 = 0;
        double f_0 = Math.log(0.1);
        double x_1 = 10;
        double y_1 = 0;
        double f_1 = Math.log(10);
        int max_iterations = 1000;

        DoubleBinaryOperator func = (x, y) -> {
            return Math.log(x);
        };

        // make sure we're passing the right bounds
        assertEquals(f_0, func.applyAsDouble(x_0, y_0), kDelta);
        assertEquals(f_1, func.applyAsDouble(x_1, y_1), kDelta);

        double s = Math100.findRoot(func, x_0, y_0, f_0, x_1, y_1, f_1, max_iterations);

        assertEquals(0.091, s, kDelta);
    }



}
