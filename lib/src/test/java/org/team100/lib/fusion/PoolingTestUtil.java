package org.team100.lib.fusion;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class PoolingTestUtil {
    private static final double kDelta = 0.001;

    static RandomVector<N1> v1(double x, double P) {
        Matrix<N1, N1> xV = VecBuilder.fill(x);
        Variance<N1> PV = new Variance<>(VecBuilder.fill(P));
        return new RandomVector<>(xV, PV);
    }

    static AngularRandomVector<N1> a1(double x, double P) {
        Matrix<N1, N1> xV = VecBuilder.fill(x);
        Variance<N1> PV = new Variance<>(VecBuilder.fill(P));
        return new AngularRandomVector<>(xV, PV);
    }

    static void assert1(RandomVector<N1> v, double x, double P) {
        assertArrayEquals(new double[] { x }, v.x.getData(), kDelta);
        assertArrayEquals(new double[] { P }, v.Kxx.getData(), kDelta);
    }

    static Matrix<N2, N2> m2(double x00, double x01, double x10, double x11) {
        Matrix<N2, N2> X = new Matrix<>(Nat.N2(), Nat.N2());
        X.set(0, 0, x00);
        X.set(0, 1, x01);
        X.set(1, 0, x10);
        X.set(1, 1, x11);
        return X;
    }

    static RandomVector<N2> v2(double x0, double x1,
            double P00, double P01, double P10, double P11) {
        Matrix<N2, N1> xV = VecBuilder.fill(x0, x1);
        Variance<N2> PV = new Variance<>(m2(P00, P01, P10, P11));
        return new RandomVector<>(xV, PV);
    }

    static void assert2(RandomVector<N2> v, double x0, double x1,
            double P00, double P01, double P10, double P11) {
        assertArrayEquals(new double[] { x0, x1 }, v.x.getData(), kDelta);
        assertArrayEquals(new double[] { P00, P01, P10, P11 }, v.Kxx.getData(), kDelta);
    }
}
