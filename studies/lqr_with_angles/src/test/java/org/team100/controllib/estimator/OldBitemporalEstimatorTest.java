package org.team100.controllib.estimator;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.Map.Entry;

import org.junit.jupiter.api.Test;
import org.team100.controllib.fusion.LinearPooling;
import org.team100.controllib.fusion.VarianceWeightedLinearPooling;
import org.team100.controllib.math.MeasurementUncertainty;
import org.team100.controllib.math.RandomVector;
import org.team100.controllib.math.Variance;
import org.team100.controllib.math.WhiteNoiseVector;
import org.team100.controllib.storage.BitemporalBuffer;
import org.team100.controllib.system.NonlinearPlant;
import org.team100.controllib.system.examples.DoubleIntegratorCartesian1D;
import org.team100.controllib.system.examples.NoisyLimitedPlant1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class OldBitemporalEstimatorTest {
    private static final double kDelta = 0.001;
    private static boolean kPrint = false;

    @Test
    public void testStopped() {
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        NonlinearPlant<N2, N1, N2> plant = new DoubleIntegratorCartesian1D(w,v);
        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(plant);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(plant);

        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();

        BitemporalBuffer<RandomVector<N2>> stateBuffer = new BitemporalBuffer<>(1000);
        // no motion
        Variance<N2> p = Variance.from2StdDev(0.316228,0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        stateBuffer.put(0l, 0.0,
                new RandomVector<N2>(
                        VecBuilder.fill(0, 0),
                        p));
        OldBitemporalEstimator<N2, N1, N2> bitemporalEstimator = new OldBitemporalEstimator<>(plant, stateBuffer, predictor,
                pointEstimator, pooling);
        {
            // this should find the initial state
            RandomVector<N2> prediction = bitemporalEstimator.predict(VecBuilder.fill(0), 0, 0);
            assertAll(
                    () -> assertEquals(0, prediction.x.get(0, 0), kDelta),
                    () -> assertEquals(0, prediction.x.get(1, 0), kDelta));
        }
        {
            // since there is no motion this should be the same
            RandomVector<N2> prediction = bitemporalEstimator.predict(VecBuilder.fill(0), 10, 1.0);
            assertAll(
                    () -> assertEquals(0, prediction.x.get(0, 0), kDelta),
                    () -> assertEquals(0, prediction.x.get(1, 0), kDelta));
        }
    }

    @Test
    public void testMotion() {
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        // note different uncertainties here
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.1,0.1);
        NonlinearPlant<N2, N1, N2> plant = new DoubleIntegratorCartesian1D(w,v);
        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(plant);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(plant);

        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();

        BitemporalBuffer<RandomVector<N2>> stateBuffer = new BitemporalBuffer<>(1000);
        // velocity = 1
        Variance<N2> p = Variance.from2StdDev(0.316228,0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        stateBuffer.put(0l, 0.0,
                new RandomVector<N2>(
                        VecBuilder.fill(0, 1),
                        p));
        OldBitemporalEstimator<N2, N1, N2> bitemporalEstimator = new OldBitemporalEstimator<>(plant, stateBuffer, predictor,
                pointEstimator, pooling);
        {
            // this should find the initial state
            RandomVector<N2> prediction = bitemporalEstimator.predict(VecBuilder.fill(0), 0, 0);
            assertAll(
                    () -> assertEquals(0, prediction.x.get(0, 0), kDelta),
                    () -> assertEquals(1, prediction.x.get(1, 0), kDelta));
        }
        {
            // with v=1 we should be at p=1 after 1 sec
            RandomVector<N2> prediction = bitemporalEstimator.predict(VecBuilder.fill(0), 10, 1.0);
            assertAll(
                    () -> assertEquals(1, prediction.x.get(0, 0), kDelta),
                    () -> assertEquals(1, prediction.x.get(1, 0), kDelta));
        }
        {
            // extrapolate really far, should be at p=100
            RandomVector<N2> prediction = bitemporalEstimator.predict(VecBuilder.fill(0), 10, 100.0);
            assertAll(
                    () -> assertEquals(100, prediction.x.get(0, 0), kDelta),
                    () -> assertEquals(1, prediction.x.get(1, 0), kDelta));
        }
    }

    /** correct position and velocity separately every 0.005s */
    @Test
    public void testCorrection() {
        if (kPrint) System.out.println("SEPARATE");
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        NoisyLimitedPlant1D plant = new DoubleIntegratorCartesian1D(w,v);
        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(plant);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(plant);

        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        BitemporalBuffer<RandomVector<N2>> stateBuffer = new BitemporalBuffer<>(1000);
        long recordTime = 0l;
        double validTime = 0;
        // TODO: realistic covariance
        Variance<N2> p = Variance.from2StdDev(0.316228,0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        stateBuffer.put(0l, 0.0,
                new RandomVector<N2>(
                        VecBuilder.fill(0, 1),
                        p));

        OldBitemporalEstimator<N2, N1, N2> bitemporalEstimator = new OldBitemporalEstimator<>(plant, stateBuffer, predictor,
                pointEstimator, pooling);
        RandomVector<N2> xhat;// = estimator.getXhat();
        // Matrix<N2, N2> P = estimator.getP();
        // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
        // validTime, xhat.get(0, 0),
        // xhat.get(1, 0), P.get(0, 0), P.get(0, 1), P.get(1, 0), P.get(1, 1));

        for (long i = 10; i < 1000; i += 10) {
            recordTime = i;
            validTime = 0.001 * i;
            xhat = bitemporalEstimator.correct(plant.position(1), recordTime, validTime);
            // P = estimator.getP();
            // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
            // validTime, xhat.get(0, 0),
            // xhat.get(1, 0), P.get(0, 0), P.get(0, 1), P.get(1, 0), P.get(1, 1));

            recordTime += 5;
            validTime += 0.005;
            xhat = bitemporalEstimator.correct(plant.velocity(0), recordTime, validTime);
            // P = estimator.getP();
            // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
            // validTime, xhat.get(0, 0),
            // xhat.get(1, 0), P.get(0, 0), P.get(0, 1), P.get(1, 0), P.get(1, 1));
        }

        // final correction
        xhat = bitemporalEstimator.correct(plant.position(1), 1000l, 1.0);
        xhat = bitemporalEstimator.correct(plant.velocity(0),  1010l, 1.005);
        // these are now the same because the time-step is the same fixed number as
        // below (0.01 s)
        // these are the EKF values
        // assertEquals(0.814, xhat.get(0, 0), kDelta);
        // assertEquals(1.435, xhat.get(1, 0), kDelta);
        // these are the new values; a little different because of constant (a little
        // higher) gain.
        // assertEquals(0.945, xhat.x.get(0, 0), kDelta);
        assertEquals(1, xhat.x.get(0, 0), kDelta);
        // assertEquals(1.655, xhat.x.get(1, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);
    }

    private RandomVector<N2> y2(double y0, double y1) {
        Variance<N2> p = Variance.from2StdDev(0.316228, 0.316228);        
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        return new RandomVector<>(VecBuilder.fill(y0, y1), p);
    }

    /**
     * correct position and velocity together every 0.01s.
     * the combined update works faster because of the dt scaling.
     * there's a copy of this test in ExtendedKalmanFilterTest to verify the
     * behavior is the same.
     */
    @Test
    public void testFullCorrection() {
        if (kPrint)
            System.out.println("FULL MY VERSION");
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        NoisyLimitedPlant1D plant = new DoubleIntegratorCartesian1D(w,v);

        // so what *should* happen with variance?
        Matrix<N2, N2> m_contQ = new Matrix<>(Nat.N2(), Nat.N2());
        m_contQ.set(0, 0, 0.0001);
        m_contQ.set(1, 1, 0.0001);

        assertArrayEquals(new double[] { 0.0001, 0, 0, 0.0001 }, m_contQ.getData(), 0.0001);

        Matrix<N2, N2> m_contR = new Matrix<>(Nat.N2(), Nat.N2());
        m_contR.set(0, 0, 0.01);
        m_contR.set(1, 1, 0.01);

        assertArrayEquals(new double[] { 0.01, 0, 0, 0.01 }, m_contR.getData(), 0.0001);

        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(plant);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(plant);

        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        BitemporalBuffer<RandomVector<N2>> stateBuffer = new BitemporalBuffer<>(1000);
        long recordTime = 0l;
        double validTime = 0;

        // initial xhat is zero with Q covariance
        // TODO is this a reasonable first guess?
        RandomVector<N2> xhat = new RandomVector<>(new Matrix<>(Nat.N2(), Nat.N1()), new Variance<>(m_contQ));
        assertArrayEquals(new double[] { 0, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.0001, 0, 0, 0.0001 }, xhat.Kxx.getData(), 0.0001);

        stateBuffer.put(recordTime, validTime, xhat);
        OldBitemporalEstimator<N2, N1, N2> bitemporalEstimator = new OldBitemporalEstimator<>(plant, stateBuffer, predictor,
                pointEstimator, pooling);

        if (kPrint)
            System.out.println(" time, xhat0, xhat1,     p00,     p01,     p10,     p11");
        if (kPrint)
            System.out.printf("%5.3f, %5.3f, %5.3f, %7.5f, %7.5f, %7.5f, %7.5f\n",
                    validTime, xhat.x.get(0, 0),
                    xhat.x.get(1, 0), xhat.Kxx.get(0, 0), xhat.Kxx.get(0, 1), xhat.Kxx.get(1, 0), xhat.Kxx.get(1, 1));

        // measure position 1, velocity 0
        RandomVector<N2> y = y2(1, 0);

        for (long i = 10; i < 1000; i += 10) {
            recordTime = i;
            validTime = 0.001 * i;
            xhat = bitemporalEstimator.correct(y, recordTime, validTime);
            if (kPrint)
                System.out.printf("%5.3f, %5.3f, %5.3f, %7.5f, %7.5f, %7.5f, %7.5f\n",
                        validTime, xhat.x.get(0, 0),
                        xhat.x.get(1, 0), xhat.Kxx.get(0, 0), xhat.Kxx.get(0, 1), xhat.Kxx.get(1, 0), xhat.Kxx.get(1, 1));
        }

        // final correction
        xhat = bitemporalEstimator.correct(y,  1000l, 1.0);
        // the real EKF, using fixed time-step 0.01 s
        // assertArrayEquals(new double[]{0.296, 0.06},xhat.x.getData(),kDelta);
        // these are the new ones, a little higher because of the higher constant gain.
        // assertArrayEquals(new double[] { 0.344, 0.072 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.999, 0 }, xhat.x.getData(), kDelta);

        // the real EKF
        // assertArrayEquals(new double[] { 0.00008, 0.00014, 0.00014, 0.00368 },
        // P.getData(), 0.00001);
        // P never changes which is WRONG WRONG WRONG
        // assertArrayEquals(new double[] { 0.00428, 0.00091, 0.00091, 0.00042 },
        // xhat.P.getData(), 0.0001);

        // by the end this is pretty dang sure
        assertArrayEquals(new double[] { 0.001, 0, 0, 0.0001 }, xhat.Kxx.getData(), 0.0001);
    }

    @Test
    public void testFullCorrectionAndPrediction() {
        if (kPrint)
            System.out.println("FULL MY VERSION CORRECT AND PREDICT");
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        NoisyLimitedPlant1D plant = new DoubleIntegratorCartesian1D(w,v);

        // so what *should* happen with variance?
        // Matrix<N2, N2> m_contQ = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(),
        // plant.stdev());
        Matrix<N2, N2> m_contQ = new Matrix<>(Nat.N2(), Nat.N2());
        m_contQ.set(0, 0, 0.0001);
        m_contQ.set(1, 1, 0.0001);
        assertArrayEquals(new double[] { 0.0001, 0, 0, 0.0001 }, m_contQ.getData(), 0.0001);

        // Matrix<N2, N2> m_contR = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(),
        // plant.full().stdev());
        Matrix<N2, N2> m_contR = new Matrix<>(Nat.N2(), Nat.N2());
        m_contR.set(0, 0, 0.01);
        m_contR.set(1, 1, 0.01);
        assertArrayEquals(new double[] { 0.01, 0, 0, 0.01 }, m_contR.getData(), 0.0001);
        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(plant);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(plant);

        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        BitemporalBuffer<RandomVector<N2>> stateBuffer = new BitemporalBuffer<>(1000);
        long recordTime = 0l;
        double validTime = 0;

        // initial xhat is zero with Q covariance
        // TODO is this a reasonable first guess?
        RandomVector<N2> xhat = new RandomVector<>(new Matrix<>(Nat.N2(), Nat.N1()), new Variance<>(m_contQ));
        assertArrayEquals(new double[] { 0, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.0001, 0, 0, 0.0001 }, xhat.Kxx.getData(), 0.0001);
        // DARE provides this:
        // assertArrayEquals(new double[] { 0.0043, 0.0009, 0.0009, 0.0004 },
        // xhat.P.getData(), 0.0001);

        stateBuffer.put(recordTime, validTime, xhat);
        OldBitemporalEstimator<N2, N1, N2> bitemporalEstimator = new OldBitemporalEstimator<>(plant, stateBuffer, predictor,
                pointEstimator, pooling);

        if (kPrint)
            System.out.println(" time, xhat0, xhat1,     p00,     p01,     p10,     p11");
        if (kPrint)
            System.out.printf("%5.3f, %5.3f, %5.3f, %7.5f, %7.5f, %7.5f, %7.5f\n",
                    validTime, xhat.x.get(0, 0),
                    xhat.x.get(1, 0), xhat.Kxx.get(0, 0), xhat.Kxx.get(0, 1), xhat.Kxx.get(1, 0), xhat.Kxx.get(1, 1));

        // no control input
        Matrix<N1, N1> u = VecBuilder.fill(0);
        // measure position 1, velocity 0
        RandomVector<N2> y = y2(1, 0);

        for (long i = 10; i < 1000; i += 10) {
            recordTime = i;
            validTime = 0.001 * i;

            // show both prediction and correction

            // TODO this doesn't yet inject noise, i think.
            xhat = bitemporalEstimator.predict(u, recordTime, validTime);
            if (kPrint)
                System.out.printf("%5.3f, %5.3f, %5.3f, %7.5f, %7.5f, %7.5f, %7.5f predict\n",
                        validTime, xhat.x.get(0, 0),
                        xhat.x.get(1, 0), xhat.Kxx.get(0, 0), xhat.Kxx.get(0, 1), xhat.Kxx.get(1, 0), xhat.Kxx.get(1, 1));

            // there is no "u" here because the measurement function "h" is required to be
            // u-invariant
            xhat = bitemporalEstimator.correct(y, recordTime, validTime);
            if (kPrint)
                System.out.printf("%5.3f, %5.3f, %5.3f, %7.5f, %7.5f, %7.5f, %7.5f correct\n",
                        validTime, xhat.x.get(0, 0),
                        xhat.x.get(1, 0), xhat.Kxx.get(0, 0), xhat.Kxx.get(0, 1), xhat.Kxx.get(1, 0), xhat.Kxx.get(1, 1));
        }

        // final correction
        xhat = bitemporalEstimator.correct(y, 1000l, 1.0);
        // the real EKF, using fixed time-step 0.01 s
        // assertArrayEquals(new double[]{1.052,1.435},xhat.x.getData(),kDelta);
        // prediction makes this more similar to EKF than the correction-only one above
        // assertArrayEquals(new double[] { 0.377, 0.071 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.999, 0 }, xhat.x.getData(), kDelta);

        // the real EKF
        // assertArrayEquals(new double[] { 0.00008, 0.00014, 0.00014, 0.00368 },
        // P.getData(), 0.00001);
        // P never changes which is WRONG WRONG WRONG
        // assertArrayEquals(new double[] { 0.00428, 0.00091, 0.00091, 0.00042 },
        // xhat.P.getData(), 0.0001);
        // by the end this is pretty dang sure
        assertArrayEquals(new double[] { 0.001, 0, 0, 0.00001 }, xhat.Kxx.getData(), 0.0001);
    }

    @Test
    public void testFloor() {
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        NoisyLimitedPlant1D plant = new DoubleIntegratorCartesian1D(w,v);
        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(plant);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(plant);

        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();

        BitemporalBuffer<RandomVector<N2>> stateBuffer = new BitemporalBuffer<>(1000);
        OldBitemporalEstimator<N2, N1, N2> bitemporalEstimator = new OldBitemporalEstimator<>(plant, stateBuffer, predictor,
                pointEstimator, pooling);
        {
            // uninitialized
            Throwable exception = assertThrows(IllegalStateException.class, () -> bitemporalEstimator.floor(1));
            assertEquals("No floor key (not initialized?): 1.0", exception.getMessage());
        }

        // TODO: realistic covariance
        stateBuffer.put(0l, 0.0, new RandomVector<N2>(VecBuilder.fill(0, 0), Variance.zero2()));
        {
            Entry<Double, Entry<Long, RandomVector<N2>>> floor = bitemporalEstimator.floor(0);
            RandomVector<N2> x = floor.getValue().getValue();
            double t = floor.getKey();
            assertAll(
                    () -> assertEquals(0, x.x.get(0, 0), kDelta),
                    () -> assertEquals(0, x.x.get(1, 0), kDelta),
                    () -> assertEquals(0, t, kDelta));
        }

        {
            Throwable exception = assertThrows(IllegalArgumentException.class, () -> bitemporalEstimator.floor(-1));
            assertEquals("Negative time is not allowed: -1.0", exception.getMessage());
        }
        {
            // just one state in the buffer
            Entry<Double, Entry<Long, RandomVector<N2>>> floor = bitemporalEstimator.floor(10);
            RandomVector<N2> x = floor.getValue().getValue();
            double t = floor.getKey();
            assertAll(
                    () -> assertEquals(0, x.x.get(0, 0), kDelta),
                    () -> assertEquals(0, x.x.get(1, 0), kDelta),
                    () -> assertEquals(0, t, kDelta));
        }

    }

}
