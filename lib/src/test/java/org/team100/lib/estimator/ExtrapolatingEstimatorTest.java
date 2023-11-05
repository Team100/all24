package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.system.MockNonlinearPlant;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

class ExtrapolatingEstimatorTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    static RandomVector<N1> v1(double x, double P) {
        Matrix<N1, N1> xV = VecBuilder.fill(x);
        Matrix<N1, N1> PV = VecBuilder.fill(P);
        return new RandomVector<>(xV, new Variance<>(PV));
    }

    static void assert1(RandomVector<N1> v, double x, double P) {
        assertArrayEquals(new double[] { x }, v.x.getData(), kDelta);
        assertArrayEquals(new double[] { P }, v.Kxx.getData(), kDelta);
    }

    static public class f1ZeroPlant extends MockNonlinearPlant<N1, N1, N1> {
        @Override
        public RandomVector<N1> f(RandomVector<N1> x, Matrix<N1, N1> u) {
            return new RandomVector<>(VecBuilder.fill(0),
            Variance.fromStdDev(Nat.N1(),VecBuilder.fill(1)));
        }
    }

    /** xdot = 0 */
    public RandomVector<N1> f1Zero(RandomVector<N1> x, Matrix<N1, N1> u) {
        return new RandomVector<>(VecBuilder.fill(0),
        Variance.fromStdDev(Nat.N1(),VecBuilder.fill(1)));
    }

    static public class f1xxPlant extends MockNonlinearPlant<N1, N1, N1> {

        @Override
        public RandomVector<N1> f(RandomVector<N1> x, Matrix<N1, N1> u) {
            Variance<N1> p =  Variance.fromStdDev(Nat.N1(),VecBuilder.fill(1));
            // p.set(0, 0, 1);
            return new RandomVector<>(x.x, p);
        }
    }

    /** xdot = x */
    public RandomVector<N1> f1X(RandomVector<N1> x, Matrix<N1, N1> u) {
        Variance<N1> p = Variance.fromStdDev(Nat.N1(),VecBuilder.fill(1));
        // p.set(0, 0, 1);
        return new RandomVector<>(x.x, p);
        // return x;
    }

    @Test
    void testZero() {
        NonlinearPlant<N1, N1, N1> plant = new f1ZeroPlant();
        ExtrapolatingEstimator<N1, N1, N1> p = new ExtrapolatingEstimator<>(plant);
        Matrix<N1, N1> u = VecBuilder.fill(0);
        double dtS = 1;
        // if xdot is zero then the prediction is always the same as the input
        {
            RandomVector<N1> x = v1(0, 0);
            RandomVector<N1> v = p.predict(x, u, dtS);
            assert1(v, 0, 0.278);
        }
        {
            RandomVector<N1> x = v1(1, 1);
            RandomVector<N1> v = p.predict(x, u, dtS);
            assert1(v, 1, 1.278);
        }
        {
            RandomVector<N1> x = v1(2, 2);
            RandomVector<N1> v = p.predict(x, u, dtS);
            assert1(v, 2, 2.278);
        }
    }

    @Test
    void testX() {
        NonlinearPlant<N1, N1, N1> plant = new f1xxPlant();

        ExtrapolatingEstimator<N1, N1, N1> p = new ExtrapolatingEstimator<>(plant);
        Matrix<N1, N1> u = VecBuilder.fill(0);
        double dtS = 1;
        {
            // if x is zero, then nothing changes
            RandomVector<N1> x = v1(0, 0);
            RandomVector<N1> v = p.predict(x, u, dtS);
            assert1(v, 0, 0.278);
        }
        {
            // if x is one, then the prediction should be e, and it's pretty close.
            RandomVector<N1> x = v1(1, 1);
            RandomVector<N1> v = p.predict(x, u, dtS);
            assert1(v, 2.708, 1.278); // should be 2.718
        }
        {
            RandomVector<N1> x = v1(2, 2);
            RandomVector<N1> v = p.predict(x, u, dtS);
            assert1(v, 5.417, 2.278);
        }
    }

    static public class f1Plant extends MockNonlinearPlant<N1, N1, N1> {
        @Override
        public RandomVector<N1> f(RandomVector<N1> x, Matrix<N1, N1> u) {
            Matrix<N1, N1> xx = new Matrix<>(Nat.N1(), Nat.N1());
            Variance<N1> p = Variance.fromStdDev(Nat.N1(),VecBuilder.fill(1));
            // p.set(0, 0, 1);
            return new RandomVector<>(xx, p);
        }
    }

    static public class f1PlantWithNoise extends f1Plant  {
        @Override
        public WhiteNoiseVector<N1> w() {
            return new WhiteNoiseVector<>(new Variance<>(VecBuilder.fill(1)));
        }
    }

    public RandomVector<N1> f1(RandomVector<N1> x, Matrix<N1, N1> u) {
        Matrix<N1, N1> xx = new Matrix<>(Nat.N1(), Nat.N1());
        Variance<N1> p = Variance.fromStdDev(Nat.N1(),VecBuilder.fill(1));
        return new RandomVector<>(xx, p);
    }

    static public class f1xPlant extends MockNonlinearPlant<N1, N1, N1> {

        @Override
        public RandomVector<N1> f(RandomVector<N1> x, Matrix<N1, N1> u) {
            Variance<N1> p = Variance.fromStdDev(Nat.N1(),VecBuilder.fill(1));
            return new RandomVector<>(x.x, p);
        }

    }

    public RandomVector<N1> f1x(RandomVector<N1> x, Matrix<N1, N1> u) {
        Variance<N1> p = Variance.fromStdDev(Nat.N1(),VecBuilder.fill(1));
        return new RandomVector<>(x.x, p);
    }

    static public class f2Plant extends MockNonlinearPlant<N2, N1, N2> {

        @Override
        public RandomVector<N2> f(RandomVector<N2> x, Matrix<N1, N1> u) {
            Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
            Variance<N2> p = Variance.from2StdDev(1, 1);
            // p.set(0, 0, 1);
            // p.set(1, 1, 1);
            return new RandomVector<>(xx, p);
        }

    }
    static public class f2PlantWithNoise extends f2Plant  {
        @Override
        public WhiteNoiseVector<N2> w() {
            Matrix<N2, N2> pp = new Matrix<>(Nat.N2(), Nat.N2());
            pp.set(0, 0, 2);
            pp.set(0, 1, 0);
            pp.set(1, 0, 0);
            pp.set(1, 1, 0.5);
            return new WhiteNoiseVector<>(new Variance<>(pp));
        }
    }

    public RandomVector<N2> f2(RandomVector<N2> x, Matrix<N1, N1> u) {
        Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
        Variance<N2> p = Variance.from2StdDev(1, 1);
        // p.set(0, 0, 1);
        // p.set(1, 1, 1);
        return new RandomVector<>(xx, p);
    }

    @Test
    void testRandomVectorIntegration1() {
        Matrix<N1, N1> x = new Matrix<>(Nat.N1(), Nat.N1());
        x.set(0, 0, 1);
        Variance<N1> p = Variance.fromStdDev(Nat.N1(),VecBuilder.fill(1));
        // p.set(0, 0, 1);
        RandomVector<N1> v1 = new RandomVector<>(x, p);
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        // big time step here to see the effect
        NonlinearPlant<N1, N1, N1> plant = new f1Plant();
        ExtrapolatingEstimator<N1, N1, N1> predictor = new ExtrapolatingEstimator<>(plant);
        RandomVector<N1> i1 = predictor.predict(v1, u, 1.0);
        // same as input
        assertArrayEquals(new double[] { 1 }, i1.x.getData(), kDelta);
        // more variance over time
        assertArrayEquals(new double[] { 1.278 }, i1.Kxx.getData(), kDelta);
    }

    @Test
    void testRandomVectorIntegration1WithNoise() {
        Matrix<N1, N1> x = new Matrix<>(Nat.N1(), Nat.N1());
        x.set(0, 0, 1);
        Variance<N1> p = Variance.fromStdDev(Nat.N1(), VecBuilder.fill(1));
        // p.set(0, 0, 1);
        RandomVector<N1> v1 = new RandomVector<>(x, p);
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        // big time step here to see the effect
        // WhiteNoiseVector<N1> xi = new WhiteNoiseVector<>(VecBuilder.fill(1));
        NonlinearPlant<N1, N1, N1> plant = new f1PlantWithNoise();
        ExtrapolatingEstimator<N1, N1, N1> predictor = new ExtrapolatingEstimator<>(plant);
        RandomVector<N1> i1 = predictor.predictWithNoise(v1, u, 1.0);
        // same as input
        assertArrayEquals(new double[] { 1 }, i1.x.getData(), kDelta);
        // noise adds "t" worth of extra variance compared to the above case
        assertArrayEquals(new double[] { 2.278 }, i1.Kxx.getData(), kDelta);
    }

    @Test
    void testRandomVectorIntegration1x() {
        Matrix<N1, N1> x = new Matrix<>(Nat.N1(), Nat.N1());
        x.set(0, 0, 1);

        Variance<N1> p = Variance.fromStdDev(Nat.N1(), VecBuilder.fill(1));
        // p.set(0, 0, 1);
        RandomVector<N1> v1 = new RandomVector<>(x, p);
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        // big time step here to see the effect
        NonlinearPlant<N1, N1, N1> plant = new f1xPlant();

        ExtrapolatingEstimator<N1, N1, N1> predictor = new ExtrapolatingEstimator<>(plant);
        RandomVector<N1> i1 = predictor.predict(v1, u, 1);
        // pretty close to e, which is the right answer.
        assertArrayEquals(new double[] { 2.708 }, i1.x.getData(), kDelta);
        // more variance over time
        assertArrayEquals(new double[] { 1.278 }, i1.Kxx.getData(), kDelta);
    }

    @Test
    void testRandomVectorIntegration2() {
        Matrix<N2, N1> x = new Matrix<>(Nat.N2(), Nat.N1());
        x.set(0, 0, 1);
        x.set(1, 0, 1);
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 1);
        p.set(0, 1, 0);
        p.set(1, 0, 0);
        p.set(1, 1, 1);
        RandomVector<N2> v2 = new RandomVector<>(x, new Variance<>(p));
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        // big time step here to see the effect
        NonlinearPlant<N2, N1, N2> plant = new f2Plant();
        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(plant);
        RandomVector<N2> i2 = predictor.predict(v2, u, 1);
        assertArrayEquals(new double[] { 1, 1 }, i2.x.getData(), kDelta);
        assertArrayEquals(new double[] { 1.278, 0, 0, 1.278 }, i2.Kxx.getData(), kDelta);
    }

    @Test
    void testRandomVectorIntegration2WithNoise() {
        Matrix<N2, N1> x = new Matrix<>(Nat.N2(), Nat.N1());
        x.set(0, 0, 1);
        x.set(1, 0, 1);
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 1);
        p.set(0, 1, 0);
        p.set(1, 0, 0);
        p.set(1, 1, 1);
        RandomVector<N2> v2 = new RandomVector<>(x, new Variance<>(p));
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());

        // Matrix<N2, N2> pp = new Matrix<>(Nat.N2(), Nat.N2());
        // pp.set(0, 0, 2);
        // pp.set(0, 1, 0);
        // pp.set(1, 0, 0);
        // pp.set(1, 1, 0.5);

        // WhiteNoiseVector<N2> xi = new WhiteNoiseVector<>(pp);
        // big time step here to see the effect
        NonlinearPlant<N2, N1, N2> plant = new f2PlantWithNoise();
        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(plant);
        RandomVector<N2> i2 = predictor.predictWithNoise(v2, u, 1);
        assertArrayEquals(new double[] { 1, 1 }, i2.x.getData(), kDelta);
        // noise variance varies by dimension; 0th is larger, 1st is smaller
        assertArrayEquals(new double[] { 3.278, 0, 0, 1.778 }, i2.Kxx.getData(), kDelta);
    }

    @Test
    void testObserverWrappingPredictOnly() {
        // just test the observer prediction across the boundary
        // it just predicts over and over.
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary

        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(w,v);
        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(system);

        Variance<N2> p = Variance.from2StdDev(0.316228,0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);

        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertArrayEquals(new double[] { -3.132, 0 }, xhat.x.getData(), kDelta);

        // saturate negative-going
        final Matrix<N1, N1> u = VecBuilder.fill(-12);

        xhat = predictor.predict(xhat, u, kDt);
        assertArrayEquals(new double[] { -3.134, -0.240 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);
        assertArrayEquals(new double[] { -3.141, -0.480 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);
        assertArrayEquals(new double[] { 3.130, -0.720 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);
        assertArrayEquals(new double[] { 3.113, -0.960 }, xhat.x.getData(), kDelta);
    }

    @Test
    void testCoastWrapping() {
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(w,v);
        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(system);

        Variance<N2> p = Variance.from2StdDev(0.316228, 0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        Vector<N2> x = VecBuilder.fill(-1.0 * Math.PI + 0.01, -10);
        RandomVector<N2> xhat = new AngularRandomVector<>(x, p);
        assertArrayEquals(new double[] { -3.132, -10 }, xhat.x.getData(), kDelta);

        final Matrix<N1, N1> u = VecBuilder.fill(0);

        xhat = predictor.predict(xhat, u, kDt);
        assertArrayEquals(new double[] { 2.952, -10 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);
        assertArrayEquals(new double[] { 2.752, -10 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);
        assertArrayEquals(new double[] { 2.552, -10 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);
        assertArrayEquals(new double[] { 2.352, -10 }, xhat.x.getData(), kDelta);

    }

    @Test
    void testIntegration1() {

        NonlinearPlant<N1,N1,N1> system = new MockNonlinearPlant<>() {
            @Override
            public WhiteNoiseVector<N1> w() {
                Matrix<N1, N1> p = new Matrix<>(Nat.N1(), Nat.N1());
                p.set(0, 0, 2);
                return new WhiteNoiseVector<>(new Variance<>(p));
            }
        };

        ExtrapolatingEstimator<N1, N1, N1> predictor = new ExtrapolatingEstimator<>(system);
        RandomVector<N1> x = new RandomVector<>(VecBuilder.fill(0), Variance.zero(Nat.N1()));
        x = predictor.addNoise(x, 0.02);
        assertArrayEquals(new double[] { 0 }, x.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.04 }, x.Kxx.getData(), kDelta);
    }

    @Test
    void testIntegration2() {
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(w,v) {
            @Override
            public WhiteNoiseVector<N2> w() {
                Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
                p.set(0, 0, 2);
                p.set(0, 1, 0);
                p.set(1, 0, 0);
                p.set(1, 1, 0.5);
                return new WhiteNoiseVector<>(new Variance<>(p));
            }
        };

        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(system);
        RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(0, 0), Variance.zero2());
        x = predictor.addNoise(x, 0.02);
        assertArrayEquals(new double[] { 0, 0 }, x.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.04, 0, 0, 0.01 }, x.Kxx.getData(), kDelta);
    }

}
