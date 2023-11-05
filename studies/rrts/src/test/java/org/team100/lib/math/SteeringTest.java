package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.BiFunction;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;

/** Experiment with steering to a near node. */
public class SteeringTest {
    /** The top level is just a 2d double-integrator. */
    BiFunction<Matrix<N4, N1>, Matrix<N2, N1>, Matrix<N4, N1>> f = (x, u) -> {
        double x2 = x.get(1, 0);
        double y2 = x.get(3, 0);
        double ux = u.get(0, 0);
        double uy = u.get(1, 0);
        double x1dot = x2;
        double x2dot = ux;
        double y1dot = y2;
        double y2dot = uy;
        Matrix<N4, N1> result = new Matrix<>(Nat.N4(), Nat.N1());
        result.set(0, 0, x1dot);
        result.set(1, 0, x2dot);
        result.set(2, 0, y1dot);
        result.set(3, 0, y2dot);
        return result;
    };

    /** 1d double-integrator. */
    BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f2 = (x, u) -> {
        double x2 = x.get(1, 0);
        double ux = u.get(0, 0);
        double x1dot = x2;
        double x2dot = ux;
        Matrix<N2, N1> result = new Matrix<>(Nat.N2(), Nat.N1());
        result.set(0, 0, x1dot);
        result.set(1, 0, x2dot);
        return result;
    };

    /**
     * double integrator result is
     * x1 = x1_0 + x2_0 * dt + 0.5 * u * dt^2
     * x2 = x2_0 + u * dt
     * 
     * note that for dt << 1, drop the squared term and get
     * x1 = x1_0 + x2_0 * dt
     * x2 = x2_0 + u * dt
     */
    @Test
    void testIntegration() {
        Matrix<N4, N1> x1 = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 0, 0, 0, 0 });
        Matrix<N2, N1> u = new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 1, 0 });
        double dt = 1;
        Matrix<N4, N1> x2 = NumericalIntegration.rk4(f, x1, u, dt);
        assertArrayEquals(new double[] { 0.5, 1, 0, 0 }, x2.getData(), 0.001);
    }

    @Test
    void testIntegration2() {
        Matrix<N4, N1> x1 = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1, 1, 0, 0 });
        Matrix<N2, N1> u = new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 1, 0 });
        double dt = 1;
        Matrix<N4, N1> x2 = NumericalIntegration.rk4(f, x1, u, dt);
        assertArrayEquals(new double[] { 2.5, 2, 0, 0 }, x2.getData(), 0.001);
    }

    /**
     * x and y are independent
     */
    @Test
    void testIntegration3() {
        Matrix<N4, N1> x1 = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 0, 0, 0, 0 });
        Matrix<N2, N1> u = new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 1, 1 });
        double dt = 1;
        Matrix<N4, N1> x2 = NumericalIntegration.rk4(f, x1, u, dt);
        assertArrayEquals(new double[] { 0.5, 1, 0.5, 1 }, x2.getData(), 0.001);
    }

    /**
     * for a double integrator there is a unique constant (u, dt) solution
     * to any (x1, x2) pair. but this solution isn't the minimum-time solution.
     */
    @Test
    void testSolve() {
        Matrix<N2, N1> x1 = new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 0, 0 });
        Matrix<N2, N1> x2 = new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 0.5, 1.0 });
        double x2_1 = x1.get(1, 0);
        Matrix<N2, N1> dx = x2.minus(x1);
        double dx1 = dx.get(0, 0);
        double dx2 = dx.get(1, 0);
        double uu = (x2_1 * dx2 + 0.5 * dx2 * dx2) / dx1;
        double dt = dx2 / uu;
        assertEquals(1, uu, 0.001);
        assertEquals(1, dt, 0.001);
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1(), new double[] { uu });
        Matrix<N2, N1> xx2 = NumericalIntegration.rk4(f2, x1, u, dt);
        assertArrayEquals(new double[] { 0.5, 1 }, xx2.getData(), 0.001);
    }
}
