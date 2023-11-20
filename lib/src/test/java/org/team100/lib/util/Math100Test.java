package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

// passes with these uncommented
// import com.acmerobotics.roadrunner.util.MathUtil;

import org.junit.jupiter.api.Test;

class Math100Test {
    private static final double kDelta = 0.001;

    @Test
    void testEpsilonEquals() {
        assertTrue(Math100.epsilonEquals(1,1));
        assertFalse(Math100.epsilonEquals(1,1.01));
    }

    @Test
    void testSolveQuadraticTwo() {
        List<Double> roots = Math100.solveQuadratic(1,0,-1);
        assertEquals(2, roots.size());
        assertEquals(1, roots.get(0), kDelta);
        assertEquals(-1, roots.get(1), kDelta);
    }

    @Test
    void testSolveQuadraticOne() {
        List<Double> roots = Math100.solveQuadratic(1,2,1);
        assertEquals(1, roots.size());
        assertEquals(-1, roots.get(0), kDelta);
    }
    
    @Test
    void testSolveQuadraticZero() {
        List<Double> roots = Math100.solveQuadratic(1,0,1);
        assertEquals(0, roots.size());
    }
}
