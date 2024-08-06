package org.team100.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Arrays;
import java.util.StringJoiner;
import java.util.function.DoubleUnaryOperator;
import java.util.stream.DoubleStream;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.jni.EigenJNI;

class RBFInterpolatorTest {
    private static final double kDelta = 1e-6;

    /**
     * Make sure I know what the solver is doing.
     */
    @Test
    void testSolver() {
        // row-major
        double[] A = new double[] { 1, 0, 0, 1 };
        int Arows = 2;
        int Acols = 2;
        double[] B = new double[] { 1, 0, 0, 1 };
        int Brows = 2;
        int Bcols = 2;
        // dst has same dimensions as B
        double[] dst = new double[4];
        EigenJNI.solveFullPivHouseholderQr(A, Arows, Acols, B, Brows, Bcols, dst);
        assertEquals("[1.0, 0.0, 0.0, 1.0]", Arrays.toString(dst));
    }

    /**
     * Verify the construction of the kernel matrix, $\Phi$ for identity RBF,
     * i.e. verify the distance calculation in the matrix.
     */
    @Test
    void testPhiDistance() {
        DoubleUnaryOperator identity = r -> r;
        {
            double[][] x = { { 1, 0 } };
            double[][] phi = RBFInterpolator.phi(x, identity);
            // matrix size is equal to the number of points
            assertEquals(1, phi.length);
            assertEquals(1, phi[0].length);
            assertEquals("[[0.00]]", toString(phi));
        }
        {
            double[][] x = { { 0, 0 }, { 1, 0 } };
            double[][] phi = RBFInterpolator.phi(x, identity);
            // matrix size is equal to the number of points
            assertEquals(2, phi.length);
            assertEquals(2, phi[0].length);
            // zero on the diagonal, 1 off the diagonal
            assertEquals("[[0.00,1.00],[1.00,0.00]]", toString(phi));
        }

        {
            double[][] x = { { 0 }, { 1 }, { 2 } };
            double[][] phi = RBFInterpolator.phi(x, identity);
            // matrix size is equal to the number of points
            assertEquals(3, phi.length);
            assertEquals(3, phi[0].length);
            // zero on the diagonal, 1 off the diagonal
            assertEquals("[[0.00,1.00,2.00],[1.00,0.00,1.00],[2.00,1.00,0.00]]", toString(phi));
        }
    }

    @Test
    void testPhi() {
        {
            double[][] x = { { 1, 0 } };
            double[][] phi = RBFInterpolator.phi(x, RBFInterpolator.GAUSSIAN);
            // matrix size is equal to the number of points
            assertEquals(1, phi.length);
            assertEquals(1, phi[0].length);
            assertEquals("[[1.00]]", toString(phi));
        }
        {
            double[][] x = { { 0, 0 }, { 1, 0 } };
            double[][] phi = RBFInterpolator.phi(x, RBFInterpolator.GAUSSIAN);
            // matrix size is equal to the number of points
            assertEquals(2, phi.length);
            assertEquals(2, phi[0].length);
            // zero on the diagonal, 1 off the diagonal
            assertEquals("[[1.00,0.37],[0.37,1.00]]", toString(phi));
        }

        {
            double[][] x = { { 0 }, { 1 }, { 2 } };
            double[][] phi = RBFInterpolator.phi(x, RBFInterpolator.GAUSSIAN);
            // matrix size is equal to the number of points
            assertEquals(3, phi.length);
            assertEquals(3, phi[0].length);
            // zero on the diagonal, 1 off the diagonal
            assertEquals("[[1.00,0.37,0.02],[0.37,1.00,0.37],[0.02,0.37,1.00]]", toString(phi));
        }
    }

    private String toString(double[][] x) {
        StringJoiner b = new StringJoiner(",", "[", "]");
        for (double[] row : x) {
            StringJoiner rowJoiner = new StringJoiner(",", "[", "]");
            DoubleStream.of(row).forEach(cell -> rowJoiner.add(String.format("%.2f", cell)));
            b.add(rowJoiner.toString());
        }
        return b.toString();
    }

    @Test
    void testR() {
        {
            double[] a = { 1, 0 };
            double[] b = { 0, 1 };
            double r = RBFInterpolator.r(a, b);
            assertEquals(Math.sqrt(2), r, kDelta);
        }
        {
            double[] a = { 0, 0, 0 };
            double[] b = { 1, 1, 1 };
            double r = RBFInterpolator.r(a, b);
            assertEquals(Math.sqrt(3), r, kDelta);
        }
    }

    @Test
    void testSimple() {
        // the known independent variables
        double[][] x = { { 1.0, 2.0 }, { 3.0, 4.0 } };
        // the known values for f
        double[][] y = { { 11.0, 12.0 }, { 13.0, 14.0 } };
        DoubleUnaryOperator rbf = RBFInterpolator.GAUSSIAN;
        RBFInterpolator interp = new RBFInterpolator(x, y, rbf);

    }

}
