package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalIntegration;

/**
 * Do I need to enforce time polarity?
 * Nope, RK4 works forwards and backwards.
 */
class NumericalIntegrationTest {
    private static final double kDelta = 0.001;

    Matrix<N1, N1> constantF(Matrix<N1, N1> x, Matrix<N1, N1> u) {
        return VecBuilder.fill(1);
    }

    @Test
    void testPositiveTime() {
        Matrix<N1, N1> prevXhat = VecBuilder.fill(0);
        Matrix<N1, N1> u = VecBuilder.fill(0);
        Matrix<N1, N1> xhat = NumericalIntegration.rk4(this::constantF, prevXhat, u, 1);
        assertEquals(1, xhat.get(0, 0), kDelta);
    }

    @Test
    void testNegativeTime() {
        Matrix<N1, N1> prevXhat = VecBuilder.fill(0);
        Matrix<N1, N1> u = VecBuilder.fill(0);
        Matrix<N1, N1> xhat = NumericalIntegration.rk4(this::constantF, prevXhat, u, -1);
        // negative time means negative motion, works fine
        assertEquals(-1, xhat.get(0, 0), kDelta);
    }

    @Test
    void testRoundtripTime() {
        Matrix<N1, N1> prevXhat = VecBuilder.fill(0);
        Matrix<N1, N1> u = VecBuilder.fill(0);
        Matrix<N1, N1> xhat = NumericalIntegration.rk4(this::constantF, prevXhat, u, 1);
        // should be back where we started
        Matrix<N1, N1> newXhat = NumericalIntegration.rk4(this::constantF, xhat, u, -1);
        assertEquals(0, newXhat.get(0, 0), kDelta);
    }
}
