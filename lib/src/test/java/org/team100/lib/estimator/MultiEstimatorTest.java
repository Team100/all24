package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/** Illustrates multiple estimators and fusion. */
class MultiEstimatorTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    @Test
    void testMultipleSensors() {
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(w, v);
        ExtrapolatingEstimator<N2, N1, N2> extrapolator = new ExtrapolatingEstimator<>(system);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(system);

        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();

        Variance<N2> p = Variance.from2StdDev(0.316228, 0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        Matrix<N1, N1> u = VecBuilder.fill(-12);

        RandomVector<N2> x;

        xhat = extrapolator.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(system.position(-3.134));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(system.velocity(-0.240));
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.134, -0.240 }, xhat.x.getData(), kDelta);

        xhat = extrapolator.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(system.position(-3.141));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(system.velocity(-0.480));
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.141, -0.480 }, xhat.x.getData(), kDelta);

        xhat = extrapolator.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(system.position(3.13));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(system.velocity(-0.720));
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { 3.130, -0.720 }, xhat.x.getData(), kDelta);

        xhat = extrapolator.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(system.position(3.113));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(system.velocity(-0.960));
        xhat = pooling.fuse(x, xhat);

        assertArrayEquals(new double[] { 3.113, -0.960 }, xhat.x.getData(), kDelta);
    }

    @Test
    void testMultipleSensorsWithTrend() {
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        // 0.0001 0.01 = pretty tight measurement variances
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(w, v);
        ExtrapolatingEstimator<N2, N1, N2> extrapolator = new ExtrapolatingEstimator<>(system);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(system);
        TrendEstimator<N2, N1, N2> trendEstimator = new TrendEstimator<>(system);

        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();

        Variance<N2> p = Variance.from2StdDev(0.316228, 0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertArrayEquals(new double[] { -3.132, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.1, 0, 0, 0.1 }, xhat.Kxx.getData(), kDelta);

        Matrix<N1, N1> u = VecBuilder.fill(-12);

        RandomVector<N2> x;

        xhat = extrapolator.predict(xhat, u, kDt);
        assertArrayEquals(new double[] { -3.134, -0.240 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.1, 0, 0, 0.1 }, xhat.Kxx.getData(), kDelta);

        x = pointEstimator.stateForMeasurementWithZeroU(system.position(-3.134));
        assertArrayEquals(new double[] { -3.134, 0 }, x.x.getData(), kDelta);
        // position measurement variance is very tight
        assertArrayEquals(new double[] { 0.0001, 0, 0, 1e9 }, x.Kxx.getData(), 0.00001);

        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.134, -0.240 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.0001, 0, 0, 0.10001 }, xhat.Kxx.getData(), 0.00001);

        x = pointEstimator.stateForMeasurementWithZeroU(system.velocity(-0.240));
        assertArrayEquals(new double[] { 0, -0.240 }, x.x.getData(), kDelta);
        assertArrayEquals(new double[] { 1e9, 0, 0, 0.01 }, x.Kxx.getData(), kDelta);

        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.134, -0.240 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.0001, 0, 0, 0.00909 }, xhat.Kxx.getData(), 0.00001);

        xhat = extrapolator.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(system.position(-3.141));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(system.velocity(-0.480));
        xhat = pooling.fuse(x, xhat);

        // trend the past two observations
        x = trendEstimator.stateForMeasurementPair(u, system.position(-3.134), system.position(-3.141), kDt);
        // what does the trend say?
        // position variance is very tight so resulting velocity variance is ok
        assertArrayEquals(new double[] { 0, -0.350 }, x.x.getData(), kDelta);
        assertArrayEquals(new double[] { 1e9, 0, 0, 0.5 }, x.Kxx.getData(), kDelta);

        // it makes a tiny difference in the fused number
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.141, -0.479 }, xhat.x.getData(), kDelta);
        // variance is quite small
        assertArrayEquals(new double[] { 0.00005, 0, 0, 0.00487 }, xhat.Kxx.getData(), 0.00001);

        xhat = extrapolator.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(system.position(3.13));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(system.velocity(-0.720));
        xhat = pooling.fuse(x, xhat);

        // trend the past two observations
        x = trendEstimator.stateForMeasurementPair(u, system.position(-3.141), system.position(3.13), kDt);
        assertArrayEquals(new double[] { 0, -0.609 }, x.x.getData(), kDelta);
        assertArrayEquals(new double[] { 1e9, 0, 0, 0.5 }, x.Kxx.getData(), kDelta);

        xhat = pooling.fuse(x, xhat);
        // notice crossing the boundary
        assertArrayEquals(new double[] { 3.130, -0.718 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.000034, 0, 0, 0.00333 }, xhat.Kxx.getData(), 0.00001);

        xhat = extrapolator.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(system.position(3.113));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(system.velocity(-0.960));
        xhat = pooling.fuse(x, xhat);
        // trend the past two observations
        x = trendEstimator.stateForMeasurementPair(u, system.position(3.13), system.position(3.113), kDt);
        assertArrayEquals(new double[] { 0, -0.850 }, x.x.getData(), kDelta);
        assertArrayEquals(new double[] { 1e9, 0, 0, 0.5 }, x.Kxx.getData(), kDelta);

        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { 3.113, -0.958 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.000034, 0, 0, 0.00255 }, xhat.Kxx.getData(), 0.00001);

    }

    @Test
    void testObserverWrappingCorrectVelocityOnly() {
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        // velocity measurement variance is 0.01.
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(w, v);
        // IntegratingPredictor<N2, N1, N2> predictor = new
        // IntegratingPredictor<>(system);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(system);
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();

        // start in negative territory
        Variance<N2> p = Variance.from2StdDev(0.316228, 0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);

        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertArrayEquals(new double[] { -3.132, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.1, 0, 0, 0.1 }, xhat.Kxx.getData(), 0.00001);
        
        // check that the velocity measurement is right.
        // it has high variance for the don't-know row and low variance for the
        // measurement
        RandomVector<N2> yvel = system.velocity(-0.240);
        assertArrayEquals(new double[] { 0, -0.240, }, yvel.x.getData(), kDelta);
        assertArrayEquals(new double[] { 1e9, 0, 0, 0.01 }, yvel.Kxx.getData(), kDelta);

        // measurement state is exactly the measurement
        RandomVector<N2> x = pointEstimator.stateForMeasurementWithZeroU(yvel);
        assertArrayEquals(new double[] { 0, -0.24 }, x.x.getData(), kDelta);
        assertArrayEquals(new double[] { 1e9, 0, 0, 0.01 }, x.Kxx.getData(), 0.00001);

        // measurement is tighter than prior
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.132, -0.218 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.1, 0, 0, 0.01385 }, xhat.Kxx.getData(), 0.00001);

        x = pointEstimator.stateForMeasurementWithZeroU(system.velocity(-0.480));
        assertArrayEquals(new double[] { 0, -0.48 }, x.x.getData(), kDelta);
        assertArrayEquals(new double[] { 1e9, 0, 0, 0.01 }, x.Kxx.getData(), 0.00001);

        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.132, -0.370 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.1, 0, 0, 0.022498 }, xhat.Kxx.getData(), 0.00001);

        x = pointEstimator.stateForMeasurementWithZeroU(system.velocity(-0.720));
        assertArrayEquals(new double[] { 0, -0.72}, x.x.getData(), kDelta);
        assertArrayEquals(new double[] { 1e9, 0, 0, 0.01 }, x.Kxx.getData(), 0.00001);

        // so the uncorrected position is left untouched
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.132, -0.612 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.1, 0, 0, 0.032984 }, xhat.Kxx.getData(), 0.00001);

    }

    @Test
    void testObserverWrappingCorrectPositionOnly() {
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        // measurement of position is very tight
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(w, v);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(system);
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();

        // start in negative territory, a little positive of -PI.
        // variance is much wider than the measurement
        Variance<N2> p = Variance.from2StdDev(0.316228, 0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);

        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertArrayEquals(new double[] { -3.132, 0 }, xhat.x.getData(), kDelta);

        // correct with a position that's actually on the other side of the boundary
        RandomVector<N2> x = pointEstimator.stateForMeasurementWithZeroU(system.position(-3.3));
        assertArrayEquals(new double[] { 2.983, 0 }, x.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.0001, 0, 0, 1e9 }, x.Kxx.getData(), 0.00001);

        xhat = pooling.fuse(x, xhat);
        // fused position respects the precise measurement over the loose prior
        assertArrayEquals(new double[] { 2.983, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.00013, 0, 0, 0.1 }, xhat.Kxx.getData(), 0.00001);

        x = pointEstimator.stateForMeasurementWithZeroU(system.position(-3.5));
        assertArrayEquals(new double[] { 2.783, 0 }, x.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.0001, 0, 0, 1e9 }, x.Kxx.getData(), 0.00001);

        // now the two updates have about the same variance so the mean is near the
        // middle
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { 2.871, 0 }, xhat.x.getData(), kDelta);
        // and the variance grows quite a bit because of the dispersion in means
        // (and the variance in the "don't know" value stays the same the whole time)
        assertArrayEquals(new double[] { 0.00992, 0, 0, 0.1 }, xhat.Kxx.getData(), 0.00001);

    }

    @Test
    void testObserverWrappingPredictAndCorrect() {
        // just test the observer across the boundary
        // with both predict and correct
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(w, v);
        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(system);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(system);
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();

        // initially, state estimate: near -pi, motionless
        Variance<N2> p = Variance.from2StdDev(0.316228, 0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);

        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // saturate negative-going
        final Matrix<N1, N1> u = VecBuilder.fill(-12);

        RandomVector<N2> x;

        xhat = predictor.predict(xhat, u, kDt);
        x = pointEstimator.stateForMeasurementWithZeroU(system.position(-3.134));
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.134, -0.240 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);
        x = pointEstimator.stateForMeasurementWithZeroU(system.position(-3.141));
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.141, -0.480, }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);
        x = pointEstimator.stateForMeasurementWithZeroU(system.position(3.13));
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { 3.130, -0.720 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);
        x = pointEstimator.stateForMeasurementWithZeroU(system.position(3.113));
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { 3.113, -0.960 }, xhat.x.getData(), kDelta);
    }

}
