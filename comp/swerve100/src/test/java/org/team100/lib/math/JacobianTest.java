package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;
import org.team100.lib.system.examples.Pendulum1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

class JacobianTest {
    private static final double kDelta = 0.001;
    private static final RandomVector<N2> xZero = new RandomVector<>(VecBuilder.fill(0, 0), Variance.zero2());

    public RandomVector<N2> f(RandomVector<N2> x, Matrix<N1, N1> u) {
        return x;
    }

    public RandomVector<N2> h(RandomVector<N2> x, Matrix<N1, N1> u) {
        return x;
    }

    @Test
    void testJacobianF() {
        RandomVector<N2> x = new RandomVector<>(new Matrix<>(Nat.N2(), Nat.N1()), Variance.zero2());
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        Matrix<N2, N2> A = Jacobian.numericalJacobianX(Nat.N2(), Nat.N2(), this::f, x, u);
        assertArrayEquals(new double[] { 1, 0, 0, 1 }, A.getData(), kDelta);
    }

    @Test
    void testJacobianH() {
        // TODO: make an H version
        RandomVector<N2> x = new RandomVector<>(new Matrix<>(Nat.N2(), Nat.N1()), Variance.zero2());
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        Matrix<N2, N2> A = Jacobian.numericalJacobianX(Nat.N2(), Nat.N2(), this::h, x, u);
        assertArrayEquals(new double[] { 1, 0, 0, 1 }, A.getData(), kDelta);
    }

    @Test
    void testJacobianU() {
        RandomVector<N2> x = new RandomVector<>(new Matrix<>(Nat.N2(), Nat.N1()), Variance.zero2());
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        Matrix<N2, N1> B = Jacobian.numericalJacobianU(Nat.N2(), Nat.N1(), this::f, x, u);
        assertArrayEquals(new double[] { 0, 0 }, B.getData(), kDelta);
    }

    /**
     * A = [0 1 0 0] constant
     */
    @Test
    void testDoubleIntegratorA() {
        Nat<N2> rows = Nat.N2();
        Nat<N2> states = Nat.N2();
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        NonlinearPlant<N2, N1, N2> plant = new DoubleIntegratorRotary1D(w,v);
        {
            // at zero
            RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(0, 0), Variance.zero2());
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N2> A = Jacobian.numericalJacobianX(rows, states, plant::f, x, u);
            assertArrayEquals(new double[] { 0, 1, 0, 0 }, A.getData(), kDelta);
            Matrix<N1, N1> uu = plant.finvWrtU(x, xZero);
            assertArrayEquals(new double[] { 0 }, uu.getData(), kDelta);
        }
        {
            // different position, same A
            RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(1, 0), Variance.zero2());
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N2> A = Jacobian.numericalJacobianX(rows, states, plant::f, x, u);
            assertArrayEquals(new double[] { 0, 1, 0, 0 }, A.getData(), kDelta);
            Matrix<N1, N1> uu = plant.finvWrtU(x, xZero);
            assertArrayEquals(new double[] { 0 }, uu.getData(), kDelta);
        }
        {
            // different u, same A
            RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(0, 0), Variance.zero2());
            Matrix<N1, N1> u = VecBuilder.fill(10000);
            Matrix<N2, N2> A = Jacobian.numericalJacobianX(rows, states, plant::f, x, u);
            assertArrayEquals(new double[] { 0, 1, 0, 0 }, A.getData(), kDelta);
            Matrix<N1, N1> uu = plant.finvWrtU(x, xZero);
            assertArrayEquals(new double[] { 0 }, uu.getData(), kDelta);
        }
    }

    /**
     * B = [0 1] constant
     */
    @Test
    void testDoubleIntegratorB() {
        Nat<N2> rows = Nat.N2();
        Nat<N1> inputs = Nat.N1();
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        NonlinearPlant<N2, N1, N2> plant = new DoubleIntegratorRotary1D(w,v);
        {
            // at zero
            RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(0, 0), Variance.zero2());
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N1> B = Jacobian.numericalJacobianU(rows, inputs, plant::f, x, u);
            assertArrayEquals(new double[] { 0, 1 }, B.getData(), kDelta);
            Matrix<N1, N1> uu = plant.finvWrtU(x, xZero);
            assertArrayEquals(new double[] { 0 }, uu.getData(), kDelta);
        }
        {
            // different position, same B
            RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(1, 0), Variance.zero2());
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N1> B = Jacobian.numericalJacobianU(rows, inputs, plant::f, x, u);
            assertArrayEquals(new double[] { 0, 1 }, B.getData(), kDelta);
            Matrix<N1, N1> uu = plant.finvWrtU(x, xZero);
            assertArrayEquals(new double[] { 0 }, uu.getData(), kDelta);
        }
        {
            // different u, same B
            RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(0, 0), Variance.zero2());
            Matrix<N1, N1> u = VecBuilder.fill(10000);
            Matrix<N2, N1> B = Jacobian.numericalJacobianU(rows, inputs, plant::f, x, u);
            assertArrayEquals(new double[] { 0, 1 }, B.getData(), kDelta);
            Matrix<N1, N1> uu = plant.finvWrtU(x, xZero);
            assertArrayEquals(new double[] { 0 }, uu.getData(), kDelta);
        }
    }

    /**
     * A = [0 1 sin(p) 0]
     */
    @Test
    void testPendulumA() {
        Nat<N2> rows = Nat.N2();
        Nat<N2> states = Nat.N2();
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        NonlinearPlant<N2, N1, N2> plant = new Pendulum1D(w,v);
        {
            // at zero degrees (max gravity) the gravity force doesn't change much with position
            RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(0, 0), Variance.zero2());
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N2> A = Jacobian.numericalJacobianX(rows, states, plant::f, x, u);
            assertArrayEquals(new double[] { 0, 1, 0, 0 }, A.getData(), kDelta);
            Matrix<N1, N1> uu = plant.finvWrtU(x, xZero);
            assertArrayEquals(new double[] { 1 }, uu.getData(), kDelta);
        }
        {
            // at 30 degrees the gravity force changes a little with position
            RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(Math.PI / 6, 0), Variance.zero2());
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N2> A = Jacobian.numericalJacobianX(rows, states, plant::f, x, u);
            assertArrayEquals(new double[] { 0, 1, 0.5, 0 }, A.getData(), kDelta);
            Matrix<N1, N1> uu = plant.finvWrtU(x, xZero);
            assertArrayEquals(new double[] { 0.866 }, uu.getData(), kDelta);
        }
        {
            // at 60 degrees the gravity force changes a lot with position
            RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(Math.PI / 3, 0), Variance.zero2());
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N2> A = Jacobian.numericalJacobianX(rows, states, plant::f, x, u);
            assertArrayEquals(new double[] { 0, 1, 0.866, 0 }, A.getData(), kDelta);
            Matrix<N1, N1> uu = plant.finvWrtU(x, xZero);
            assertArrayEquals(new double[] { 0.5 }, uu.getData(), kDelta);
        }
        {
            // at 90 degrees the gravity force is about proportional with position
            RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(Math.PI / 2, 0), Variance.zero2());
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N2> A = Jacobian.numericalJacobianX(rows, states, plant::f, x, u);
            assertArrayEquals(new double[] { 0, 1, 1, 0 }, A.getData(), kDelta);
            Matrix<N1, N1> uu = plant.finvWrtU(x, xZero);
            assertArrayEquals(new double[] { 0 }, uu.getData(), kDelta);
        }
    }

    /**
     * B = [0 1] constant because while the model is nonlinear the control response
     * is linear.
     */
    @Test
    void testPendulumB() {
        Nat<N2> rows = Nat.N2();
        Nat<N1> inputs = Nat.N1();
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        NonlinearPlant<N2, N1, N2> plant = new Pendulum1D(w,v);
        {
            // at zero
            RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(0, 0), Variance.zero2());
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N1> B = Jacobian.numericalJacobianU(rows, inputs, plant::f, x, u);
            assertArrayEquals(new double[] { 0, 1 }, B.getData(), kDelta);
            Matrix<N1, N1> uu = plant.finvWrtU(x, xZero);
            assertArrayEquals(new double[] { 1 }, uu.getData(), kDelta);

        }
        {
            // different position, same B
            RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(1, 0), Variance.zero2());
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N1> B = Jacobian.numericalJacobianU(rows, inputs, plant::f, x, u);
            assertArrayEquals(new double[] { 0, 1 }, B.getData(), kDelta);
            Matrix<N1, N1> uu = plant.finvWrtU(x, xZero);
            // this is cos(1)
            assertArrayEquals(new double[] { 0.540 }, uu.getData(), kDelta);
        }
    }

}
