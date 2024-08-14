package org.team100.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.StringJoiner;
import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleUnaryOperator;
import java.util.stream.DoubleStream;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

import java.util.List;

/** Duplicates RBFInterpolatorTest */
class RBFInterpolatingMapTest {
    private static final double kDelta = 1e-6;

    @Test
    void testPhiDistance() {
        DoubleUnaryOperator identity = r -> r;
        {
            List<Vector<N2>> x = List.of(
                    VecBuilder.fill(1, 0));
            double[][] phi = RBFInterpolatingMap.phi(x, identity);
            // matrix size is equal to the number of points
            assertEquals(1, phi.length);
            assertEquals(1, phi[0].length);
            assertEquals("[[0.00]]", toString(phi));
        }
        {

            List<Vector<N2>> x = List.of(
                    VecBuilder.fill(0, 0),
                    VecBuilder.fill(1, 0));
            double[][] phi = RBFInterpolatingMap.phi(x, identity);
            // matrix size is equal to the number of points
            assertEquals(2, phi.length);
            assertEquals(2, phi[0].length);
            // zero on the diagonal, 1 off the diagonal
            assertEquals("[[0.00, 1.00], [1.00, 0.00]]", toString(phi));
        }

        {
            List<Vector<N1>> x = List.of(
                    VecBuilder.fill(0),
                    VecBuilder.fill(1),
                    VecBuilder.fill(2));
            double[][] phi = RBFInterpolatingMap.phi(x, identity);
            // matrix size is equal to the number of points
            assertEquals(3, phi.length);
            assertEquals(3, phi[0].length);
            // zero on the diagonal, 1 off the diagonal
            assertEquals("[[0.00, 1.00, 2.00], [1.00, 0.00, 1.00], [2.00, 1.00, 0.00]]", toString(phi));
        }
    }

    @Test
    void testPhi() {
        {
            List<Vector<N2>> x = List.of(
                    VecBuilder.fill(1, 0));
            double[][] phi = RBFInterpolatingMap.phi(x, RBFInterpolatingMap.GAUSSIAN);
            // matrix size is equal to the number of points
            assertEquals(1, phi.length);
            assertEquals(1, phi[0].length);
            assertEquals("[[1.00]]", toString(phi));
        }
        {
            List<Vector<N2>> x = List.of(
                    VecBuilder.fill(0, 0),
                    VecBuilder.fill(1, 0));
            double[][] phi = RBFInterpolatingMap.phi(x, RBFInterpolatingMap.GAUSSIAN);
            // matrix size is equal to the number of points
            assertEquals(2, phi.length);
            assertEquals(2, phi[0].length);
            // zero on the diagonal, 1 off the diagonal
            assertEquals("[[1.00, 0.37], [0.37, 1.00]]", toString(phi));
        }
        {
            List<Vector<N1>> x = List.of(
                    VecBuilder.fill(0),
                    VecBuilder.fill(1),
                    VecBuilder.fill(2));
            double[][] phi = RBFInterpolatingMap.phi(x, RBFInterpolatingMap.GAUSSIAN);
            // matrix size is equal to the number of points
            assertEquals(3, phi.length);
            assertEquals(3, phi[0].length);
            // zero on the diagonal, 1 off the diagonal
            assertEquals("[[1.00, 0.37, 0.02], [0.37, 1.00, 0.37], [0.02, 0.37, 1.00]]", toString(phi));
        }
    }

    @Test
    void testPhiVec() {
        Vector<N2> p = VecBuilder.fill(1.0, 2.0);
        List<Vector<N2>> x = List.of(
                VecBuilder.fill(0.0, 1.0),
                VecBuilder.fill(2.0, 3.0));
        double[] phiVec = RBFInterpolatingMap.phiVec(p, x, r -> r);
        assertEquals("[1.41, 1.41]", toString(phiVec));
    }

    @Test
    void testR() {
        {
            Vector<N2> a = VecBuilder.fill(1, 0);
            Vector<N2> b = VecBuilder.fill(0, 1);
            double r = RBFInterpolatingMap.r(a, b);
            assertEquals(Math.sqrt(2), r, kDelta);
        }
        {
            Vector<N3> a = VecBuilder.fill(0, 0, 0);
            Vector<N3> b = VecBuilder.fill(1, 1, 1);
            double r = RBFInterpolatingMap.r(a, b);
            assertEquals(Math.sqrt(3), r, kDelta);
        }
    }

    @Test
    void testRowMajor() {
        double[][] x = { { 1, 2 }, { 3, 4 } };
        double[] xRowMajor = RBFInterpolatingMap.rowMajor(x);
        assertEquals("[1.00, 2.00, 3.00, 4.00]", toString(xRowMajor));
    }

    @Test
    void testTwoD() {
        double[] xRowMajor = { 1, 2, 3, 4 };
        double[][] x = RBFInterpolatingMap.twoD(xRowMajor, 2, 2);
        assertEquals("[[1.00, 2.00], [3.00, 4.00]]", toString(x));
    }

    @Test
    void testR1R1OneExample() {
        // this should all work with scalars.
        // the known independent variable, 1 example in R^1
        List<Vector<N1>> x = List.of(VecBuilder.fill(1.0));
        // the known value for f, 1 examples in R^1
        List<Vector<N1>> y = List.of(VecBuilder.fill(11.0));
        RBFInterpolatingMap<N1, N1> interp = new RBFInterpolatingMap<>(
                RBFInterpolatingMap.GAUSSIAN, Nat.N1());
        interp.put(x.get(0), y.get(0));
        interp.calculateWeights();
        // since the gaussian evaluates to 1 at r=0, the weight should be 11
        assertEquals("[[11.00]]", toString(interp.m_w));
        // if we give it exactly x0, we should get back y0.
        Vector<N1> s = interp.get(VecBuilder.fill(1.0));
        assertEquals("[11.00]", toString(s));
    }

    @Test
    void testR1R1TwoExamples() {
        // this should all work with scalars.
        // the known independent variable, 1 example in R^1
        List<Vector<N1>> x = List.of(
                VecBuilder.fill(1.0),
                VecBuilder.fill(2.0));
        // the known value for f, 1 examples in R^1
        List<Vector<N1>> y = List.of(
                VecBuilder.fill(11.0),
                VecBuilder.fill(9.0));
        RBFInterpolatingMap<N1, N1> interp = new RBFInterpolatingMap<>(
                RBFInterpolatingMap.GAUSSIAN, Nat.N1());
        interp.put(x.get(0), y.get(0));
        interp.put(x.get(1), y.get(1));
        interp.calculateWeights();
        // ???
        assertEquals("[[8.89], [5.73]]", toString(interp.m_w));

        Vector<N1> p = VecBuilder.fill(1.0);
        double[] phiVec = RBFInterpolatingMap.phiVec(p, interp.m_x, interp.m_rbf);
        // rbf evaluated for r=0 and r=1
        assertEquals("[1.00, 0.37]", toString(phiVec));

        {
            // if we give it exactly x0, we should get back y0.
            assertEquals("[11.00]", toString(
                    interp.get(VecBuilder.fill(1.0))));
            assertEquals("[9.00]", toString(
                    interp.get(VecBuilder.fill(2.0))));
        }
        // note, in between isn't ideal :)
        {
            for (double xi = 1.0; xi <= 2.01; xi += 0.05) {
                Vector<N1> s = interp.get(
                        VecBuilder.fill(xi));
                System.out.printf("%5.3f %5.3f\n", xi, s.get(0));
            }
        }

    }

    @Test
    void testR2R2OneExample() {
        // the known independent variable, 1 example in R^2
        List<Vector<N2>> x = List.of(
                VecBuilder.fill(1.0, 2.0));
        // the known value for f, 1 example in R^2
        List<Vector<N2>> y = List.of(
                VecBuilder.fill(5.0, -3.0));
        RBFInterpolatingMap<N2, N2> interp = new RBFInterpolatingMap<>(
                RBFInterpolatingMap.GAUSSIAN, Nat.N2());
        interp.put(x.get(0), y.get(0));
        interp.calculateWeights();
        assertEquals("[[5.00, -3.00]]", toString(interp.m_w));
        // if we give it exactly x0, we should get back y0.
        Vector<N2> s = interp.get(VecBuilder.fill(1.0, 2.0));
        assertEquals("[5.00, -3.00]", toString(s));
    }

    @Test
    void testSimple() {
        RBFInterpolatingMap<N3, N2> interp = new RBFInterpolatingMap<>(
                RBFInterpolatingMap.GAUSSIAN, Nat.N2());
        // the known independent variables, 2 examples in R^3
        // the known values for f, 2 examples in R^2

        interp.put(VecBuilder.fill(1.0, 2.0, 3.0), VecBuilder.fill(11.0, 12.0));
        interp.put(VecBuilder.fill(4.0, 5.0, 6.0), VecBuilder.fill(13.0, 14.0));

        // assertEquals("", toString(interp.m_w));
        // interpolation means hitting the training exactly
        // so if we give it exactly x0, we should get back y0.
        Vector<N2> s = interp.get(VecBuilder.fill(1.0, 2.0, 3.0));
        assertEquals("[11.00, 12.00]", toString(s));
    }

    @Test
    void testASimpleFunction() {
        DoubleBinaryOperator fn = (x, y) -> x + y;
        int x0Range = 20;
        int x1Range = 20;
        int n = x0Range * x1Range;
        RBFInterpolatingMap<N2, N1> interp = new RBFInterpolatingMap<>(
                RBFInterpolatingMap.GAUSSIAN, Nat.N1());

        for (int x0i = 0; x0i < x0Range; x0i++) {
            for (int x1i = 0; x1i < x1Range; x1i++) {
                double x0 = -1 + x0i * 0.1;
                double x1 = -1 + x1i * 0.1;
                double y0 = fn.applyAsDouble(x0, x1);
                interp.put(
                        VecBuilder.fill(x0, x1),
                        VecBuilder.fill(y0));
            }
        }

        // verify a few points
        verify(fn, interp, 0, 0);
        verify(fn, interp, 0.1, 0.1);
        verify(fn, interp, 0.5, 0.0);

        // look at the whole thing, beyond the training data to see Runge's phenomenon.
        // the error is zero within the convex hull of the training set. :)
        {
            long ms = System.nanoTime();
            for (double px = -2; px < 2; px += 0.2) {
                for (double py = -2; py < 2; py += 0.2) {
                    double pf = fn.applyAsDouble(px, py);
                    Vector<N1> s = interp.get(VecBuilder.fill(px, py));
                    double err = pf - s.get(0);
                    System.out.printf("%5.3f %5.3f %5.3f %5.3f %5.3f \n", px, py, pf, s.get(0), err);
                }
            }
            long ms1 = System.nanoTime();
            long et = ms1 - ms;
            double etEach = (double) et / n;
            // printing seems to take about 100 us
            // calculating fn takes 10 us
            // the interpolation itself takes about 20 us
            // which is 50% slower than the array version.  :-(
            // still, 20 us is 1/1000th of the loop time, and we only
            // need to do a handful of these per loop, so it's fine.
            System.out.printf("et %d n %d etEach (ns) %5.3f\n", et, n, etEach);
        }
        {
            long ms = System.nanoTime();
            for (double px = -2; px < 2; px += 0.2) {
                for (double py = -2; py < 2; py += 0.2) {
                    interp.get(VecBuilder.fill(px, py));
                }
            }
            long ms1 = System.nanoTime();
            long et = ms1 - ms;
            double etEach = (double) et / n;
            System.out.printf("et %d n %d etEach (ns) %5.3f\n", et, n, etEach);
        }
    }

    /////////////////////////////////

    private void verify(DoubleBinaryOperator fn,
            RBFInterpolatingMap<N2, N1> interp, double px0, double px1) {
        double pf = fn.applyAsDouble(px0, px1);
        Vector<N1> s = interp.get(VecBuilder.fill(px0, px1));
        assertEquals(pf, s.get(0), kDelta);
    }

    private String toString(double[][] x) {
        StringJoiner sj = new StringJoiner(", ", "[", "]");
        for (double[] row : x) {
            sj.add(toString(row));
        }
        return sj.toString();
    }

    private String toString(double[] row) {
        StringJoiner sj = new StringJoiner(", ", "[", "]");
        DoubleStream.of(row).forEach(cell -> sj.add(String.format("%.2f", cell)));
        return sj.toString();
    }

    private <T extends Num> String toString(Vector<T> row) {
        StringJoiner sj = new StringJoiner(", ", "[", "]");
        for (int i = 0; i < row.getNumRows(); ++i) {
            sj.add(String.format("%.2f", row.get(i)));
        }
        return sj.toString();
    }

}
