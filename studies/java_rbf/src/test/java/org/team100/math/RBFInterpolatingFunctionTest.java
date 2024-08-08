package org.team100.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.StringJoiner;
import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleUnaryOperator;
import java.util.stream.DoubleStream;

import org.junit.jupiter.api.Test;
import org.team100.math.RBFInterpolatingFunction.Stats;

/** Duplicates RBFInterpolatingMapTest */
class RBFInterpolatingFunctionTest {
    record MyScalar(double a) {
        @Override
        public String toString() {
            return String.format("[%.2f]", a);
        }
    }

    record MyPair(double a, double b) {
        @Override
        public String toString() {
            return String.format("[%.2f, %.2f]", a, b);
        }
    }

    record MyTriple(double a, double b, double c) {
        @Override
        public String toString() {
            return String.format("[%.2f, %.2f, %.2f]", a, b, c);
        }
    }

    class MyScalarAdapter extends RBFInterpolatingFunction.Adapter<MyScalar> {

        @Override
        int size() {
            return 1;
        }

        @Override
        double[] toArray(MyScalar src) {
            return new double[] { src.a };
        }

        @Override
        MyScalar fromArray(double[] src) {
            return new MyScalar(src[0]);
        }

    }

    class MyPairAdapter extends RBFInterpolatingFunction.Adapter<MyPair> {

        @Override
        int size() {
            return 2;
        }

        @Override
        double[] toArray(MyPair src) {
            return new double[] { src.a, src.b };
        }

        @Override
        MyPair fromArray(double[] src) {
            return new MyPair(src[0], src[1]);
        }

    }

    class MyTripleAdapter extends RBFInterpolatingFunction.Adapter<MyTriple> {

        @Override
        int size() {
            return 3;
        }

        @Override
        double[] toArray(MyTriple src) {
            return new double[] { src.a, src.b, src.c };
        }

        @Override
        MyTriple fromArray(double[] src) {
            return new MyTriple(src[0], src[1], src[2]);
        }

    }

    private static final double kDelta = 1e-6;

    @Test
    void testPhiDistance() {
        DoubleUnaryOperator identity = r -> r;
        {
            List<MyPair> x = List.of(
                    new MyPair(1, 0));
            MyPairAdapter xadapter = new MyPairAdapter();
            // these are like 'identity' stats
            Stats[] stats = new Stats[] { new Stats(0, 1), new Stats(0, 1) };
            double[][] phi = RBFInterpolatingFunction.phi(x, xadapter, stats, identity);
            // matrix size is equal to the number of points
            assertEquals(1, phi.length);
            assertEquals(1, phi[0].length);
            assertEquals("[[0.00]]", toString(phi));
        }
        {

            List<MyPair> x = List.of(
                    new MyPair(0, 0),
                    new MyPair(1, 0));
            MyPairAdapter xadapter = new MyPairAdapter();
            // these are like 'identity' stats
            Stats[] stats = new Stats[] { new Stats(0, 1), new Stats(0, 1) };
            double[][] phi = RBFInterpolatingFunction.phi(x, xadapter, stats, identity);
            // matrix size is equal to the number of points
            assertEquals(2, phi.length);
            assertEquals(2, phi[0].length);
            // zero on the diagonal, 1 off the diagonal
            assertEquals("[[0.00, 1.00], [1.00, 0.00]]", toString(phi));
        }

        {
            List<MyScalar> x = List.of(
                    new MyScalar(0),
                    new MyScalar(1),
                    new MyScalar(2));
            MyScalarAdapter xadapter = new MyScalarAdapter();
            Stats[] stats = new Stats[] { new Stats(0, 1) };
            double[][] phi = RBFInterpolatingFunction.phi(x, xadapter, stats, identity);
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
            List<MyPair> x = List.of(
                    new MyPair(1, 0));
            MyPairAdapter xadapter = new MyPairAdapter();
            Stats[] stats = new Stats[] { new Stats(0, 1), new Stats(0, 1) };
            double[][] phi = RBFInterpolatingFunction.phi(x, xadapter, stats, RBFInterpolatingFunction.GAUSSIAN(1.0));
            // matrix size is equal to the number of points
            assertEquals(1, phi.length);
            assertEquals(1, phi[0].length);
            assertEquals("[[1.00]]", toString(phi));
        }
        {
            List<MyPair> x = List.of(
                    new MyPair(0, 0),
                    new MyPair(1, 0));
            MyPairAdapter xadapter = new MyPairAdapter();
            Stats[] stats = new Stats[] { new Stats(0, 1), new Stats(0, 1) };
            double[][] phi = RBFInterpolatingFunction.phi(x, xadapter, stats, RBFInterpolatingFunction.GAUSSIAN(1.0));
            // matrix size is equal to the number of points
            assertEquals(2, phi.length);
            assertEquals(2, phi[0].length);
            // zero on the diagonal, 1 off the diagonal
            assertEquals("[[1.00, 0.37], [0.37, 1.00]]", toString(phi));
        }
        {
            List<MyScalar> x = List.of(
                    new MyScalar(0),
                    new MyScalar(1),
                    new MyScalar(2));
            MyScalarAdapter xadapter = new MyScalarAdapter();
            Stats[] stats = new Stats[] { new Stats(0, 1), };
            double[][] phi = RBFInterpolatingFunction.phi(x, xadapter, stats, RBFInterpolatingFunction.GAUSSIAN(1.0));
            // matrix size is equal to the number of points
            assertEquals(3, phi.length);
            assertEquals(3, phi[0].length);
            // zero on the diagonal, 1 off the diagonal
            assertEquals("[[1.00, 0.37, 0.02], [0.37, 1.00, 0.37], [0.02, 0.37, 1.00]]", toString(phi));
        }
    }

    @Test
    void testPhiVec() {
        MyPair p = new MyPair(1.0, 2.0);
        List<MyPair> x = List.of(
                new MyPair(0.0, 1.0),
                new MyPair(2.0, 3.0));
        MyPairAdapter xadapter = new MyPairAdapter();
        Stats[] stats = new Stats[] { new Stats(0, 1), new Stats(0, 1) };
        double[] phiVec = RBFInterpolatingFunction.phiVec(p, x, xadapter, stats, r -> r);
        assertEquals("[1.41, 1.41]", toString(phiVec));
    }

    @Test
    void testR() {
        {
            MyPair a = new MyPair(1, 0);
            MyPair b = new MyPair(0, 1);
            MyPairAdapter xadapter = new MyPairAdapter();
            Stats[] stats = new Stats[] { new Stats(0, 1), new Stats(0, 1) };
            double r = xadapter.r(a, b, stats);
            assertEquals(Math.sqrt(2), r, kDelta);
        }
        {
            MyTriple a = new MyTriple(0, 0, 0);
            MyTriple b = new MyTriple(1, 1, 1);
            MyTripleAdapter xadapter = new MyTripleAdapter();
            Stats[] stats = new Stats[] { new Stats(0, 1), new Stats(0, 1), new Stats(0, 1) };
            double r = xadapter.r(a, b, stats);
            assertEquals(Math.sqrt(3), r, kDelta);
        }
    }

    @Test
    void testRowMajor() {
        double[][] x = { { 1, 2 }, { 3, 4 } };
        double[] xRowMajor = RBFInterpolatingFunction.rowMajor(x);
        assertEquals("[1.00, 2.00, 3.00, 4.00]", toString(xRowMajor));
    }

    @Test
    void testTwoD() {
        double[] xRowMajor = { 1, 2, 3, 4 };
        double[][] x = RBFInterpolatingFunction.twoD(xRowMajor, 2, 2);
        assertEquals("[[1.00, 2.00], [3.00, 4.00]]", toString(x));
    }

    @Test
    void testR1R1OneExample() {
        // this should all work with scalars.
        // the known independent variable, 1 example in R^1
        List<MyScalar> x = List.of(new MyScalar(1.0));
        // the known value for f, 1 examples in R^1
        List<MyScalar> y = List.of(new MyScalar(11.0));
        MyScalarAdapter xadapter = new MyScalarAdapter();
        MyScalarAdapter yadapter = new MyScalarAdapter();

        RBFInterpolatingFunction<MyScalar, MyScalar> interp = new RBFInterpolatingFunction<>(
                x, y, xadapter, yadapter,
                RBFInterpolatingFunction.GAUSSIAN(1.0));

        // since the gaussian evaluates to 1 at r=0, the weight should be 11
        assertEquals("[[0.00]]", toString(interp.m_w));
        // if we give it exactly x0, we should get back y0.
        MyScalar s = interp.apply(new MyScalar(1.0));
        assertEquals("[11.00]", s.toString());
    }

    @Test
    void testR1R1TwoExamples() {
        // this should all work with scalars.
        // the known independent variable, 1 example in R^1
        List<MyScalar> x = List.of(
                new MyScalar(1.0),
                new MyScalar(2.0));
        // the known value for f, 1 examples in R^1
        List<MyScalar> y = List.of(
                new MyScalar(11.0),
                new MyScalar(9.0));
        MyScalarAdapter xadapter = new MyScalarAdapter();
        MyScalarAdapter yadapter = new MyScalarAdapter();
        RBFInterpolatingFunction<MyScalar, MyScalar> interp = new RBFInterpolatingFunction<>(
                x, y, xadapter, yadapter, RBFInterpolatingFunction.GAUSSIAN(1.0));
        assertEquals(1, interp.m_xstats.length);
        assertEquals(1.5, interp.m_xstats[0].mean(), kDelta);
        assertEquals(0.5, interp.m_xstats[0].stddev(), kDelta);
        assertEquals(1, interp.m_ystats.length);
        assertEquals(10.0, interp.m_ystats[0].mean(), kDelta);
        assertEquals(1.0, interp.m_ystats[0].stddev(), kDelta);
        // ???
        assertEquals("[[1.02], [-1.02]]", toString(interp.m_w));

        MyScalar p = new MyScalar(1.0);
        Stats[] stats = new Stats[] { new Stats(0, 1) };

        double[] phiVec = RBFInterpolatingFunction.phiVec(p, interp.m_x, xadapter, stats, interp.m_rbf);
        // rbf evaluated for r=0 and r=1
        assertEquals("[1.00, 0.37]", toString(phiVec));

        {
            // if we give it exactly x0, we should get back y0.
            assertEquals("[11.00]", interp.apply(new MyScalar(1.0)).toString());
            assertEquals("[9.00]", interp.apply(new MyScalar(2.0)).toString());
        }
        // note, in between isn't ideal :)
        {
            for (double xi = 1.0; xi <= 2.01; xi += 0.05) {
                MyScalar s = interp.apply(
                        new MyScalar(xi));
                System.out.printf("%5.3f %5.3f\n", xi, s.a);
            }
        }

    }

    @Test
    void testR2R2OneExample() {
        // the known independent variable, 1 example in R^2
        List<MyPair> x = List.of(
                new MyPair(1.0, 2.0));
        // the known value for f, 1 example in R^2
        List<MyPair> y = List.of(
                new MyPair(5.0, -3.0));
        MyPairAdapter xadapter = new MyPairAdapter();
        MyPairAdapter yadapter = new MyPairAdapter();
        RBFInterpolatingFunction<MyPair, MyPair> interp = new RBFInterpolatingFunction<>(
                x, y, xadapter, yadapter,
                RBFInterpolatingFunction.GAUSSIAN(1.0));

        assertEquals(2, interp.m_xstats.length);
        assertEquals(1.0, interp.m_xstats[0].mean(), kDelta);
        // this is the minimum
        assertEquals(0.1, interp.m_xstats[0].stddev(), kDelta);
        assertEquals(2.0, interp.m_xstats[1].mean(), kDelta);
        // this is the minimum
        assertEquals(0.1, interp.m_xstats[1].stddev(), kDelta);
        assertEquals(2, interp.m_ystats.length);
        assertEquals(5.0, interp.m_ystats[0].mean(), kDelta);
        assertEquals(0.1, interp.m_ystats[0].stddev(), kDelta);
        assertEquals(-3.0, interp.m_ystats[1].mean(), kDelta);
        assertEquals(0.1, interp.m_ystats[1].stddev(), kDelta);

        // weight is always zero if there's just one normalized example
        assertEquals("[[0.00, 0.00]]", toString(interp.m_w));
        // if we give it exactly x0, we should get back y0.
        // (thanks to the stats offsets from zero)
        MyPair s = interp.apply(new MyPair(1.0, 2.0));
        assertEquals("[5.00, -3.00]", s.toString());
    }

    @Test
    void testSimple() {
        List<MyTriple> x = List.of(
                new MyTriple(1.0, 2.0, 3.0),
                new MyTriple(4.0, 5.0, 6.0));
        List<MyPair> y = List.of(
                new MyPair(11.0, 12.0),
                new MyPair(13.0, 14.0));
        MyTripleAdapter xadapter = new MyTripleAdapter();
        MyPairAdapter yadapter = new MyPairAdapter();
        RBFInterpolatingFunction<MyTriple, MyPair> interp = new RBFInterpolatingFunction<>(
                x, y, xadapter, yadapter,
                RBFInterpolatingFunction.GAUSSIAN(1.0));
        // the known independent variables, 2 examples in R^3
        // the known values for f, 2 examples in R^2

        // assertEquals("", toString(interp.m_w));
        // interpolation means hitting the training exactly
        // so if we give it exactly x0, we should get back y0.
        MyPair s = interp.apply(new MyTriple(1.0, 2.0, 3.0));
        assertEquals("[11.00, 12.00]", s.toString());
    }

    @Test
    void testASimpleFunction() {
        DoubleBinaryOperator fn = (x, y) -> x + y;
        int x0Range = 20;
        int x1Range = 20;
        int n = x0Range * x1Range;

        List<MyPair> x = new ArrayList<>();
        List<MyScalar> y = new ArrayList<>();
        for (int x0i = 0; x0i < x0Range; x0i++) {
            for (int x1i = 0; x1i < x1Range; x1i++) {
                double x0 = -1 + x0i * 0.1;
                double x1 = -1 + x1i * 0.1;
                double y0 = fn.applyAsDouble(x0, x1);
                x.add(new MyPair(x0, x1));
                y.add(new MyScalar(y0));
            }
        }

        MyPairAdapter xadapter = new MyPairAdapter();
        MyScalarAdapter yadapter = new MyScalarAdapter();

        RBFInterpolatingFunction<MyPair, MyScalar> interp = new RBFInterpolatingFunction<>(
                x, y, xadapter, yadapter,
                RBFInterpolatingFunction.GAUSSIAN(1.0));

        assertEquals(1, interp.m_ystats.length, kDelta);
        assertEquals(-0.1, interp.m_ystats[0].mean(), kDelta);
        assertEquals(0.815475, interp.m_ystats[0].stddev(), kDelta);

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
                    MyScalar s = interp.apply(new MyPair(px, py));
                    double err = pf - s.a;
                    System.out.printf("%5.3f %5.3f %5.3f %5.3f %5.3f \n", px, py, pf, s.a, err);
                }
            }
            long ms1 = System.nanoTime();
            long et = ms1 - ms;
            double etEach = (double) et / n;
            // printing seems to take about 100 us
            // calculating fn takes 10 us
            // the interpolation itself takes about 20 us
            // which is 50% slower than the array version. :-(
            // still, 20 us is 1/1000th of the loop time, and we only
            // need to do a handful of these per loop, so it's fine.
            System.out.printf("et %d n %d etEach (ns) %5.3f\n", et, n, etEach);
        }
        {
            long ms = System.nanoTime();
            for (double px = -2; px < 2; px += 0.2) {
                for (double py = -2; py < 2; py += 0.2) {
                    interp.apply(new MyPair(px, py));
                }
            }
            long ms1 = System.nanoTime();
            long et = ms1 - ms;
            double etEach = (double) et / n;
            System.out.printf("et %d n %d etEach (ns) %5.3f\n", et, n, etEach);
        }
    }

    @Test
    void testNormalization() {
        {
            List<MyPair> y = List.of(
                    new MyPair(11.0, 12.0),
                    new MyPair(13.0, 14.0));
            MyPairAdapter yadapter = new MyPairAdapter();
            // to normalize you need to take two passes through the data. once to get the
            // statistics, and once to normalize.
            RBFInterpolatingFunction.Stats[] stats = yadapter.stats(y);
            assertEquals(2, stats.length);
            assertEquals(12, stats[0].mean(), kDelta);
            assertEquals(1, stats[0].stddev(), kDelta);
            assertEquals(13, stats[1].mean(), kDelta);
            assertEquals(1, stats[1].stddev(), kDelta);
        }
        {
            // a problematic case to normalize
            List<MyPair> y = List.of(new MyPair(0.0, 0.0));
            MyPairAdapter yadapter = new MyPairAdapter();
            // to normalize you need to take two passes through the data. once to get the
            // statistics, and once to normalize.
            RBFInterpolatingFunction.Stats[] stats = yadapter.stats(y);
            assertEquals(2, stats.length);
            assertEquals(0, stats[0].mean(), kDelta);
            // this is the minimum
            assertEquals(0.1, stats[0].stddev(), kDelta);
            assertEquals(0, stats[1].mean(), kDelta);
            // this is the minimum
            assertEquals(0.1, stats[1].stddev(), kDelta);
        }
    }

    /////////////////////////////////

    private void verify(DoubleBinaryOperator fn,
            RBFInterpolatingFunction<MyPair, MyScalar> interp, double px0, double px1) {
        double pf = fn.applyAsDouble(px0, px1);
        MyScalar s = interp.apply(new MyPair(px0, px1));
        assertEquals(pf, s.a, kDelta);
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
}
