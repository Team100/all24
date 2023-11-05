package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.numbers.N1;

/** This is just so I understand what it does */
class AngleStatisticsTest {
    private static final double kDelta = 0.001;

    @Test
    void testWithoutWrapping() {
        Matrix<N1, N1> a = VecBuilder.fill(3.0 * Math.PI / 4);
        Matrix<N1, N1> b = VecBuilder.fill(Math.PI / 4);
        Matrix<N1, N1> c = AngleStatistics.angleResidual(a, b, 0);
        assertEquals(Math.PI / 2, c.get(0, 0), kDelta);
    }

    @Test
    void testWrapping() {
        Matrix<N1, N1> a = VecBuilder.fill(3 * Math.PI / 4);
        Matrix<N1, N1> b = VecBuilder.fill(-3 * Math.PI / 4);
        Matrix<N1, N1> c = AngleStatistics.angleResidual(a, b, 0);
        assertEquals(-1.0 * Math.PI / 2, c.get(0, 0), kDelta);
    }

}
