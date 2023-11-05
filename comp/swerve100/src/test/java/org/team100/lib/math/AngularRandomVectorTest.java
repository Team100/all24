package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

/** Verify non-Euclidean arithmetic. */
class AngularRandomVectorTest {
    private static final double kDelta = 0.001;

    @Test
    void testWrappingPlus() {
        Vector<N2> x0 = VecBuilder.fill(3, 0);
        Variance<N2> p0 = Variance.from2StdDev(0.316228, 0.316228);
        // p0.set(0, 0, 0.1);
        // p0.set(1, 1, 0.1);
        AngularRandomVector<N2> v0 = new AngularRandomVector<>(x0, p0);

        Vector<N2> x1 = VecBuilder.fill(1, 0);
        Variance<N2> p1 = Variance.from2StdDev(0.316228, 0.316228);
        // p1.set(0, 0, 0.1);
        // p1.set(1, 1, 0.1);
        AngularRandomVector<N2> v1 = new AngularRandomVector<>(x1, p1);

        // 3 + 1 should be 4-2PI
        RandomVector<N2> v2 = v0.plus(v1);
        assertTrue(v2 instanceof AngularRandomVector);
        assertArrayEquals(new double[] { -2.283, 0 }, v2.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.2, 0, 0, 0.2 }, v2.Kxx.getData(), kDelta);
    }

    @Test
    void testWrappingMinus() {
        Vector<N2> x0 = VecBuilder.fill(3, 0);
        Variance<N2> p0 = Variance.from2StdDev(0.316228, 0.316228);
        // p0.set(0, 0, 0.1);
        // p0.set(1, 1, 0.1);
        AngularRandomVector<N2> v0 = new AngularRandomVector<>(x0, p0);

        Vector<N2> x1 = VecBuilder.fill(-3, 0);
        Variance<N2> p1 = Variance.from2StdDev(0.316228, 0.316228);
        // p1.set(0, 0, 0.1);
        // p1.set(1, 1, 0.1);
        AngularRandomVector<N2> v1 = new AngularRandomVector<>(x1, p1);

        // 3 - -3  would be 6 if it were wrong but it's not, it's -0.283 (clockwise)
        RandomVector<N2> v2 = v0.minus(v1);
        assertTrue(v2 instanceof AngularRandomVector);
        assertArrayEquals(new double[] { -0.283, 0 }, v2.x.getData(), kDelta);
        // more variance here
        assertArrayEquals(new double[] { 0.2, 0, 0, 0.2 }, v2.Kxx.getData(), kDelta);
    }
}
