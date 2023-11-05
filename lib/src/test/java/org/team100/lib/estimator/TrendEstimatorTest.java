package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.system.MockNonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

class TrendEstimatorTest {
    private static final double kDelta = 0.001;

    public static class Thing extends MockNonlinearPlant<N2, N1, N2> {
        /** xdot for x */
        @Override
        public RandomVector<N2> f(RandomVector<N2> x, Matrix<N1, N1> u) {
            return x;
        }

        @Override
        public RandomVector<N2> finvWrtX(RandomVector<N2> xdot, Matrix<N1, N1> u) {
            return xdot;
        }

        /** y for x */
        @Override
        public RandomVector<N2> h(RandomVector<N2> x, Matrix<N1, N1> u) {
            return x;
        }

        /** x for y */
        @Override
        public RandomVector<N2> hinv(RandomVector<N2> y, Matrix<N1, N1> u) {
            return y;
        }
    }

    @Test
    void testStateForMeasurementPair() {
        Thing thing = new Thing();
        TrendEstimator<N2, N1, N2> trendEstimator = new TrendEstimator<>(thing);

        // y0 is 1,0 +/- 0.01
        Matrix<N2, N1> yx0 = new Matrix<>(Nat.N2(), Nat.N1());
        yx0.set(0, 0, 1);
        // Matrix<N2, N2> yP0 = new Matrix<>(Nat.N2(), Nat.N2());
        Variance<N2> yP0 = Variance.from2StdDev(0.1, 0.1);
        // yP0.set(0, 0, 0.01);
        // yP0.set(1, 1, 0.01);
        RandomVector<N2> y0 = new RandomVector<>(yx0, yP0);

        // y1 is 2,0 +/- 0.01
        Matrix<N2, N1> yx1 = new Matrix<>(Nat.N2(), Nat.N1());
        yx1.set(0, 0, 2);
        Variance<N2> yP1 = Variance.from2StdDev(0.1, 0.1);
        // yP1.set(0, 0, 0.01);
        // yP1.set(1, 1, 0.01);
        RandomVector<N2> y1 = new RandomVector<>(yx1, yP1);

        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        
        RandomVector<N2> xhat = trendEstimator.stateForMeasurementPair(u, y0, y1, 1.0);

        // for this weird system, x0dot is 1 so x0 is 1 x1dot is 0 so x1 is 0
        assertArrayEquals(new double[] { 1, 0 }, xhat.x.getData(), kDelta);
        // for this weird system there are no don't-know values, just the additive
        // variance
        assertArrayEquals(new double[] { 0.02, 0, 0, 0.02 }, xhat.Kxx.getData(), kDelta);
    }

    public static class DoubleIntegrator extends MockNonlinearPlant<N2, N1, N2> {

        /**
         * dynamics, xdot = f(x,u)
         * x0dot = x1
         * x1dot = u
         */
        @Override
        public RandomVector<N2> f(RandomVector<N2> x, Matrix<N1, N1> u) {
            Matrix<N2, N1> xdotx = new Matrix<>(Nat.N2(), Nat.N1());
            xdotx.set(0, 0, x.x.get(1, 0));
            xdotx.set(1, 0, u.get(0, 0));
            Matrix<N2, N2> xdotP = new Matrix<>(Nat.N2(), Nat.N2());
            xdotP.set(0, 0, x.Kxx.get(1, 1)); // variance from x1 becomes x0dot variance
            xdotP.set(1, 1, 0.1); // want a better estimate of response variance
            return x.make(xdotx, new Variance<>(xdotP));
        }

        /**
         * inverse of f with respect to x
         * x0 = ?
         * x1 = x0dot
         */
        @Override
        public RandomVector<N2> finvWrtX(RandomVector<N2> xdot, Matrix<N1, N1> u) {
            Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
            xx.set(1, 0, xdot.x.get(0, 0));
            Matrix<N2, N2> xP = new Matrix<>(Nat.N2(), Nat.N2());
            xP.set(0, 0, 1e9); // "don't know" variance
            xP.set(1, 1, xdot.Kxx.get(0, 0));
            return xdot.make(xx, new Variance<>(xP));
        }

        /**
         * measurement, y = h(x,u)
         * y for x
         */
        @Override
        public RandomVector<N2> h(RandomVector<N2> x, Matrix<N1, N1> u) {
            return x;
        }

        /**
         * inverse of h with respect to x
         * x for y
         */
        @Override
        public RandomVector<N2> hinv(RandomVector<N2> y, Matrix<N1, N1> u) {
            return y;
        }
    }

    @Test
    void testDoubleIntegrator() {
        DoubleIntegrator doubleIntegrator = new DoubleIntegrator();
        TrendEstimator<N2, N1, N2> trendEstimator = new TrendEstimator<>(doubleIntegrator);

        // y0 is 1,0 +/- 0.01
        Matrix<N2, N1> yx0 = new Matrix<>(Nat.N2(), Nat.N1());
        yx0.set(0, 0, 1);
        Variance<N2> yP0 = Variance.from2StdDev(0.1, 0.1);
        // yP0.set(0, 0, 0.01);
        // yP0.set(1, 1, 0.01);
        RandomVector<N2> y0 = new RandomVector<>(yx0, yP0);

        // y1 is 2,0 +/- 0.01
        Matrix<N2, N1> yx1 = new Matrix<>(Nat.N2(), Nat.N1());
        yx1.set(0, 0, 2);
        Variance<N2> yP1 = Variance.from2StdDev(0.1, 0.1);
        // yP1.set(0, 0, 0.01);
        // yP1.set(1, 1, 0.01);
        RandomVector<N2> y1 = new RandomVector<>(yx1, yP1);

        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        RandomVector<N2> xhat = trendEstimator.stateForMeasurementPair(u, y0, y1, 1.0);

        // for the double integrator there should be no estimate for position (zero)
        // but there should be an estimate of 1 for velocity
        assertArrayEquals(new double[] { 0, 1 }, xhat.x.getData(), kDelta);
        // and the variance of position should be large
        assertArrayEquals(new double[] { 1e9, 0, 0, 0.02 }, xhat.Kxx.getData(), kDelta);
    }

    @Test
    void testDoubleIntegratorSmallTimeStep() {
        DoubleIntegrator doubleIntegrator = new DoubleIntegrator();
        TrendEstimator<N2, N1, N2> trendEstimator = new TrendEstimator<>(doubleIntegrator);

        // y0 is 1,0 +/- 0.01
        Matrix<N2, N1> yx0 = new Matrix<>(Nat.N2(), Nat.N1());
        yx0.set(0, 0, 1);
        Variance<N2> yP0 = Variance.from2StdDev(0.1, 0.1);
        // yP0.set(0, 0, 0.01);
        // yP0.set(1, 1, 0.01);
        RandomVector<N2> y0 = new RandomVector<>(yx0, yP0);

        // y1 is 2,0 +/- 0.01
        Matrix<N2, N1> yx1 = new Matrix<>(Nat.N2(), Nat.N1());
        yx1.set(0, 0, 2);
        Variance<N2> yP1 = Variance.from2StdDev(0.1, 0.1);
        // yP1.set(0, 0, 0.01);
        // yP1.set(1, 1, 0.01);
        RandomVector<N2> y1 = new RandomVector<>(yx1, yP1);

        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        // note small time step here
        RandomVector<N2> xhat = trendEstimator.stateForMeasurementPair(u, y0, y1, 0.02);

        // for the double integrator there should be no estimate for position (zero)
        // but there should be an estimate of 50 for velocity
        assertArrayEquals(new double[] { 0, 50 }, xhat.x.getData(), kDelta);
        // and the variance of position should be large
        // the individual sample variance is low 0.02 but the time step is also
        // very low and it's squared.
        // so this means the trend signal will never have much effect over very small
        // time steps.  Use as wide a window as possible.
        assertArrayEquals(new double[] { 1e9, 0, 0, 50 }, xhat.Kxx.getData(), kDelta);
    }
}
