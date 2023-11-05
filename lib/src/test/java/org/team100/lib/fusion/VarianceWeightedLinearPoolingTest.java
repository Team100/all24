package org.team100.lib.fusion;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.Random;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * similar to the democratic case
 */
class VarianceWeightedLinearPoolingTest extends PoolingTestUtil {
    private static final double kDelta = 0.001;

    private static final VarianceWeightedLinearPooling<N1> p1 = new VarianceWeightedLinearPooling<N1>();
    private static final VarianceWeightedLinearPooling<N2> p2 = new VarianceWeightedLinearPooling<N2>();

    @Test
    void testUnanimity1() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(0, 1);
        RandomVector<N1> cV = p1.fuse(aV, bV);
        // see the same thing twice -> become more sure
        assert1(cV, 0, 0.5);
    }

    @Test
    void testUnanimity2() {
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 1);
        RandomVector<N2> bV = v2(0, 0, 1, 0, 0, 1);
        RandomVector<N2> cV = p2.fuse(aV, bV);
        // see the same thing twice -> become more sure
        assert2(cV, 0, 0, 0.5, 0, 0, 0.5);
    }

    @Test
    void testDifferentMeans() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 1);
        RandomVector<N1> cV = p1.fuse(aV, bV);
        // aggregate mean is right in the middle
        // aggregate variance 0.5 from dispersion, 0.25 from variance
        assert1(cV, 0.5, 0.75);
    }

    @Test
    void testDifferentVariance() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(0, 2);
        RandomVector<N1> cV = p1.fuse(aV, bV);
        // same means, so we're more sure
        assert1(cV, 0.0, 0.666);
    }

    @Test
    void testDifferent1() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 2);
        RandomVector<N1> cV = p1.fuse(aV, bV);
        // the mean is like log-linear, weighted by inverse variance
        // the variance is bigger than log-linear due to the dispersion term
        assert1(cV, 0.333, 0.888);
    }

    @Test
    void testDifferent2NoCorrelation() {
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 1);
        RandomVector<N2> bV = v2(1, 1, 2, 0, 0, 2);
        RandomVector<N2> cV = p2.fuse(aV, bV);
        // the mean is like log-linear, weighted by inverse variance
        // the variance is bigger than log-linear due to the dispersion term
        assert2(cV, 0.333, 0.333, 0.888, 0, 0, 0.888);
    }

    @Test
    void testDifferent2bNoCorrelation() {
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 2);
        RandomVector<N2> bV = v2(1, 1, 2, 0, 0, 1);
        RandomVector<N2> cV = p2.fuse(aV, bV);
        // mean leans towards the tighter variance
        assert2(cV, 0.333, 0.666, 0.888, 0, 0, 0.888);
    }

    @Test
    void testDifferent2WithCorrelation() {
        // different means, different variances
        RandomVector<N2> aV = v2(0, 0, 1, 0.5, 0.5, 1);
        RandomVector<N2> bV = v2(1, 1, 2, 0.5, 0.5, 2);
        RandomVector<N2> cV = p2.fuse(aV, bV);
        assert2(cV, 0.375, 0.375, 0.867, 0.305, 0.305, 0.867);
    }

    @Test
    void testDifferent2bWithCorrelation() {
        // some off-diagonal covariance terms and different variances
        RandomVector<N2> aV = v2(0, 0, 1, 0.5, 0.5, 2);
        RandomVector<N2> bV = v2(1, 1, 2, 0.5, 0.5, 1);
        RandomVector<N2> cV = p2.fuse(aV, bV);
        // mean leans towards the tighter variance
        // off-diagonals make this effect stronger
        assert2(cV, 0.25, 0.75, 0.922, 0.203, 0.203, 0.922);
    }

    @Test
    void testZeroVariance() {
        // can't invert this one
        RandomVector<N1> aV = v1(0, 0);
        RandomVector<N1> bV = v1(1, 1);
        // this is like the log-linear case now since it inverts the variances
        assertThrows(IllegalArgumentException.class, () -> p1.fuse(aV, bV));
    }

    @Test
    void testOKWeights() {
        // means differ by 1
        // variances are identical
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 1);
        RandomVector<N2> bV = v2(1, 1, 1, 0, 0, 1);
        Matrix<N2, N2> pa = m2(0.5, 0, 0, 0.5);
        Matrix<N2, N2> pb = m2(0.5, 0, 0, 0.5);
        RandomVector<N2> cV = p2.fuse(aV, pa, bV, pb);
        // equal weight => mean in the middle,
        // variance is 0.5 from dispersion, 0.25 from two samples
        assert2(cV, 0.5, 0.5, 0.75, 0, 0, 0.75);
    }

    @Test
    void testBadWeights() {
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 1);
        RandomVector<N2> bV = v2(1, 1, 1, 0, 0, 1);
        // these do not add to identity
        Matrix<N2, N2> pa = m2(0.5, 0, 0, 0.5);
        Matrix<N2, N2> pb = m2(1, 0, 0, 1);
        assertThrows(IllegalArgumentException.class, () -> p2.fuse(aV, pa, bV, pb));
    }

    /**
     * this method converges *much* faster than the "kalman gain" method, and it
     * seems more correct and less mysterious.
     */
    @Test
    void testIteration() {
        // initial xhat is zero
        Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
        Variance<N2> xP = Variance.from2StdDev(0.1, 0.1);
        // xP.set(0, 0, 0.01);
        // xP.set(1, 1, 0.01);
        RandomVector<N2> xhat = new RandomVector<>(xx, xP);

        // measurement is 1,0
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, 1);
        Variance<N2> yP = Variance.from2StdDev(0.1, 0.1);
        // yP.set(0, 0, 0.01);
        // yP.set(1, 1, 0.01);
        RandomVector<N2> estimateFromMeasurement = new RandomVector<>(yx, yP);

        xhat = p2.fuse(xhat, estimateFromMeasurement);
        // since the old and new have the same variance the mean is in the middle
        assertArrayEquals(new double[] { 0.5, 0 }, xhat.x.getData(), kDelta);
        // the difference in means adds to the variance but only of the first component
        assertArrayEquals(new double[] { 0.255, 0, 0, 0.005 }, xhat.Kxx.getData(), kDelta);

        xhat = p2.fuse(xhat, estimateFromMeasurement);
        // new measurement has lower variance so it is preferred
        assertArrayEquals(new double[] { 0.981, 0 }, xhat.x.getData(), kDelta);
        // mean dispersion keeps increasing P but multiple samples keep decreasing it
        assertArrayEquals(new double[] { 0.019, 0, 0, 0.003 }, xhat.Kxx.getData(), kDelta);

        xhat = p2.fuse(xhat, estimateFromMeasurement);
        assertArrayEquals(new double[] { 0.993, 0 }, xhat.x.getData(), kDelta);
        // mean dispersion is way down now
        assertArrayEquals(new double[] { 0.007, 0, 0, 0.002 }, xhat.Kxx.getData(), kDelta);

        // go all the way to the end
        for (int i = 0; i < 100; ++i) {
            xhat = p2.fuse(xhat, estimateFromMeasurement);
        }
        // we keep seeing the same mean over and over so we're really sure now
        assertArrayEquals(new double[] { 1, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.000098, 0, 0, 0.000096 }, xhat.Kxx.getData(), 0.000001);
    }

    @Test
    void testDontKnow() {
        // fuse an unknown with a known, you should just get the known back.
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 1e9);
        RandomVector<N2> bV = v2(1, 1, 1e9, 0, 0, 1);
      //  System.out.println("=========================");
        RandomVector<N2> cV = p2.fuse(aV, bV);
        assert2(cV, 0, 1, 1, 0, 0, 1);
    }

    @Test
    void testDontKnowWeights() {
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 1e9);
        RandomVector<N2> bV = v2(1, 1, 1e9, 0, 0, 1);
        Pair<Matrix<N2, N2>, Matrix<N2, N2>> weights = p2.weights(aV, bV);
        Matrix<N2, N2> pa = weights.getFirst();
        Matrix<N2, N2> pb = weights.getSecond();
        // so in the first slot since b doesn't know, we should get a
        // and in the second we should get b.
        assertArrayEquals(new double[] { 1, 0, 0, 0 }, pa.getData(), kDelta);
        assertArrayEquals(new double[] { 0, 0, 0, 1 }, pb.getData(), kDelta);
        RandomVector<N2> cV = p2.fuse(aV, pa, bV, pb);
        assert2(cV, 0, 1, 1, 0, 0, 1);
    }

    @Test
    void testWeights1a() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 1e9);
        Pair<Matrix<N1, N1>, Matrix<N1, N1>> weights = p1.weights(aV, bV);
        Matrix<N1, N1> pa = weights.getFirst();
        Matrix<N1, N1> pb = weights.getSecond();
        assertArrayEquals(new double[] { 1 }, pa.getData(), 0.000001);
        assertArrayEquals(new double[] { 0 }, pb.getData(), 0.000001);
    }

    @Test
    void testWeightPrimitive() {
        // scalar multiplication is squared by the matrix isn't? what?

        RandomVector<N1> a = v1(0, 1);
        RandomVector<N1> b = v1(1, 1e9);

        double aP = a.Kxx.getValue().get(0, 0);
        double bP = b.Kxx.getValue().get(0, 0);

        double aPI = 1.0 / aP;
        double bPI = 1.0 / bP;
        double PIsum = aPI + bPI;

        double pIsumI = 1.0 / PIsum;
        double pa = aPI * pIsumI;
        double pb = bPI * pIsumI;
        assertEquals(0.999, pa, 0.001);
        assertEquals(9.99e-10, pb, 1e-12);
    }

    @Test
    void testWeights1b() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 1);
        Pair<Matrix<N1, N1>, Matrix<N1, N1>> weights = p1.weights(aV, bV);
        Matrix<N1, N1> pa = weights.getFirst();
        Matrix<N1, N1> pb = weights.getSecond();
        assertArrayEquals(new double[] { 0.5 }, pa.getData(), kDelta);
        assertArrayEquals(new double[] { 0.5 }, pb.getData(), kDelta);
    }

    @Test
    void testVar1() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(0, 1);
        RandomVector<N1> cV = p1.fuse(aV, bV);
        assertArrayEquals(new double[] { 0 }, cV.x.getData());
        // two samples with the same mean, the underlying variance
        // is probably smaller than the sample variance
        assertArrayEquals(new double[] { 0.5 }, cV.Kxx.getData());
    }

    @Test
    void testVar1WithDispersion() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 1);
        RandomVector<N1> cV = p1.fuse(aV, bV);
        assertArrayEquals(new double[] { 0.5 }, cV.x.getData());
        // 0.5 from variance
        // each sample is 0.5 from the mean, squared is 0.25, sum is 0.5
        // so the dispersion term is 0.25
        // so total variance is 0.75
        assertArrayEquals(new double[] { 0.75 }, cV.Kxx.getData(), 0.000001);
    }

    @Test
    void testVar10() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(0, 10);
        RandomVector<N1> cV = p1.fuse(aV, bV);
        assertArrayEquals(new double[] { 0 }, cV.x.getData());
        // combine an estimate with "don't know" and you get the estimate back
        assertArrayEquals(new double[] { 0.909 }, cV.Kxx.getData(), 0.001);
    }

    @Test
    void testVar1000() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(0, 1000);
        RandomVector<N1> cV = p1.fuse(aV, bV);
        assertArrayEquals(new double[] { 0 }, cV.x.getData());
        // combine an estimate with "don't know" and you get the estimate back
        assertArrayEquals(new double[] { 0.999 }, cV.Kxx.getData(), 0.001);
    }

    //@Test
    void testRandom() {
        // make a bunch of random samples with the variance of the measurement.
        // the mean should converge to 0
        // the variance should converge to 1
        Random r = new Random(0);
        RandomVector<N1> aV = v1(r.nextGaussian(), 10);
        for (int i = 0; i < 10000; ++i) {
            aV = p1.fuse(aV, v1(r.nextGaussian(), 10));
            System.out.printf("%12.5f, %12.5f\n",
            aV.x.get(0,0),
            aV.Kxx.getValue().get(0,0));
        }
    }
}
