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

class PointEstimatorTest {
    private static final double kDelta = 0.001;

    public static class Thing extends MockNonlinearPlant<N2, N1, N2> {

        /** xdot for x */
        @Override
        public RandomVector<N2> f(RandomVector<N2> x, Matrix<N1, N1> u) {
            return x;
        }

        @Override
        public Nat<N1> inputs() {
            return Nat.N1();
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
    void testStateForFullMeasurement() {
        Thing system = new Thing();
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(system);

        // measurement is 1,0
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, 1);
        // full measurement has low variance for every row
        Variance<N2> yP = Variance.from2StdDev(0.1, 0.1);
        // yP.set(0, 0, 0.01);
        // yP.set(1, 1, 0.01);
        RandomVector<N2> y = new RandomVector<>(yx, yP);

        RandomVector<N2> xhat = pointEstimator.stateForMeasurementWithZeroU(y);
        // since the state is just the measurement,
        // you get the specified mean and variance of the measurement.
        assertArrayEquals(new double[] { 1, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.01, 0, 0, 0.01 }, xhat.Kxx.getData(), kDelta);
    }

    @Test
    void testStateForPartialMeasurement() {
        Thing system = new Thing();
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(system);

        // measurement is 1,x
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, 1);
        // partial measurement has "don't know" values
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 0.01);
        yP.set(1, 1, 1e9); // enormous variance; how big shouild this be?
        RandomVector<N2> y = new RandomVector<>(yx, new Variance<>(yP));

        RandomVector<N2> xhat = pointEstimator.stateForMeasurementWithZeroU(y);
        // since the state is just the measurement,
        // you get the specified mean and variance of the measurement.
        assertArrayEquals(new double[] { 1, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.01, 0, 0, 1e9 }, xhat.Kxx.getData(), kDelta);
    }
}
