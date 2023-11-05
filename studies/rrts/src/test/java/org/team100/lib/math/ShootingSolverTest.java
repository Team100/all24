package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.BiFunction;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;

public class ShootingSolverTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.01;

    @Test
    void testNorm() {
        // Frobenius norm does what i think it does
        Matrix<N2, N1> x = VecBuilder.fill(3, 4);
        assertEquals(5, x.normF(), DELTA);
    }

    @Test
    void testDot() {
        // matrix multiplication does what i think it does
        {
            Matrix<N2, N1> x = VecBuilder.fill(1, 0);
            Matrix<N2, N1> y = VecBuilder.fill(0, 1);
            double dot = x.transpose().times(y).get(0, 0);
            assertEquals(0, dot, DELTA);
            double xNorm = x.normF();
            double yNorm = y.normF();
            double angleRad = Math.acos(dot / (xNorm * yNorm));
            assertEquals(Math.PI / 2, angleRad, DELTA);
        }
        {
            Matrix<N2, N1> x = VecBuilder.fill(1, 0);
            Matrix<N2, N1> y = VecBuilder.fill(1, 1);
            double dot = x.transpose().times(y).get(0, 0);
            assertEquals(1, dot, DELTA);
            double xNorm = x.normF();
            double yNorm = y.normF();
            double angleRad = Math.acos(dot / (xNorm * yNorm));
            assertEquals(Math.PI / 4, angleRad, DELTA);
        }
    }

    @Test
    void testAngle() {
        ShootingSolver<N2, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 0.1, 10);
        {
            Matrix<N2, N1> x = VecBuilder.fill(1, 0);
            Matrix<N2, N1> y = VecBuilder.fill(0, 1);
            assertEquals(Math.PI / 2, s.angleRad(x, y), DELTA);
        }
        {
            Matrix<N2, N1> x = VecBuilder.fill(1, 0);
            Matrix<N2, N1> y = VecBuilder.fill(1, 1);
            assertEquals(Math.PI / 4, s.angleRad(x, y), DELTA);
        }
    }

    @Test
    void testAngleSum() {
        ShootingSolver<N2, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 0.1, 10);
        {
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 0);
            Matrix<N2, N1> x2 = VecBuilder.fill(1, 1);
            Matrix<N2, N1> minX2 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> maxX2 = VecBuilder.fill(1, 0);
            Matrix<N2, N1> x2x1 = x2.minus(x1);
            Matrix<N2, N1> x2minX2 = x2.minus(minX2);
            Matrix<N2, N1> x2maxX2 = x2.minus(maxX2);
            // 45 + 45 + 90 = pi
            assertEquals(Math.PI, s.angleSum(x2x1, x2minX2, x2maxX2));
        }
    }

    @Test
    void testAngleSum3d() {
        // 3d because i can visualize it; the actual case is 4d.
        ShootingSolver<N3, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 0.1, 10);
        {
            // this plane is x+y+z=1
            // so the closest point to 0,0,0 is 1/sqrt(3) away
            // along the inner diagonal. a cube of size 1 has
            // inner diagonal of sqrt(3) so this little cube has
            // size 1/3.
            Matrix<N3, N1> x1 = VecBuilder.fill(1, 0, 0);
            Matrix<N3, N1> x2 = VecBuilder.fill(0.333, 0.333, 0.333);
            Matrix<N3, N1> minX2 = VecBuilder.fill(0, 1, 0);
            Matrix<N3, N1> maxX2 = VecBuilder.fill(0, 0, 1);
            Matrix<N3, N1> x2x1 = x2.minus(x1);
            Matrix<N3, N1> x2minX2 = x2.minus(minX2);
            Matrix<N3, N1> x2maxX2 = x2.minus(maxX2);
            assertEquals(2.0 * Math.PI, s.angleSum(x2x1, x2minX2, x2maxX2), DELTA);
        }
    }

    @Test
    void testInside2d() {
        ShootingSolver<N2, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 0.1, 10);
        {
            // far outside
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 0);
            Matrix<N2, N1> x2 = VecBuilder.fill(1, 1);
            Matrix<N2, N1> minX2 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> maxX2 = VecBuilder.fill(1, 0);
            assertFalse(s.inside(x1, x2, minX2, maxX2));
        }
        {
            // far inside
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 0);
            Matrix<N2, N1> x2 = VecBuilder.fill(0.25, 0.25);
            Matrix<N2, N1> minX2 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> maxX2 = VecBuilder.fill(1, 0);
            assertTrue(s.inside(x1, x2, minX2, maxX2));
        }
        {
            // on the boundary, should be inside
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 0);
            Matrix<N2, N1> x2 = VecBuilder.fill(0.5, 0.5);
            Matrix<N2, N1> minX2 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> maxX2 = VecBuilder.fill(1, 0);
            assertTrue(s.inside(x1, x2, minX2, maxX2));
        }
    }

    @Test
    void testInside3d() {
        // 3d because i can visualize it; the actual case is 4d.
        ShootingSolver<N3, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 0.1, 10);
        {
            // far outside
            Matrix<N3, N1> x1 = VecBuilder.fill(0, 0, 0);
            Matrix<N3, N1> x2 = VecBuilder.fill(1, 1, 1);
            Matrix<N3, N1> minX2 = VecBuilder.fill(0, 1, 0);
            Matrix<N3, N1> maxX2 = VecBuilder.fill(1, 0, 0);
            assertFalse(s.inside(x1, x2, minX2, maxX2));
        }
        {
            // in the middle of the triangle
            Matrix<N3, N1> x1 = VecBuilder.fill(1, 0, 0);
            Matrix<N3, N1> x2 = VecBuilder.fill(0.333, 0.333, 0.333);
            Matrix<N3, N1> minX2 = VecBuilder.fill(0, 1, 0);
            Matrix<N3, N1> maxX2 = VecBuilder.fill(0, 0, 1);
            assertTrue(s.inside(x1, x2, minX2, maxX2));
        }
        {
            // near but not actually on the plane
            Matrix<N3, N1> x1 = VecBuilder.fill(1, 0, 0);
            Matrix<N3, N1> x2 = VecBuilder.fill(0.4, 0.4, 0.4);
            Matrix<N3, N1> minX2 = VecBuilder.fill(0, 1, 0);
            Matrix<N3, N1> maxX2 = VecBuilder.fill(0, 0, 1);
            assertFalse(s.inside(x1, x2, minX2, maxX2));
        }
    }

    @Test
    void testRK4MotionlessStart() {
        // double-integrator
        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = (x, u) -> {
            return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
        };
        double dt = 1;
        {
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 0);
            Matrix<N1, N1> maxU = VecBuilder.fill(1);
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, maxU, dt);
            // x = x0 + 0.5 u t^2 = 0.5
            // v = v0 + u t = 1
            assertArrayEquals(new double[] { 0.5, 1 }, maxX2.getData(), DELTA);
        }
    }

    @Test
    void testRK4MovingStart() {
        // double-integrator
        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = (x, u) -> {
            return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
        };
        Matrix<N2, N1> x1 = VecBuilder.fill(0, 1);
        {
            Matrix<N1, N1> u = VecBuilder.fill(-1);
            double dt = 1;
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, u, dt);
            // x = x0 + v t + 0.5 u t ^ 2 = 1 - 0.5 = 0.5
            // v = v0 + u = 1 - 1 = 0
            assertArrayEquals(new double[] { 0.5, 0 }, maxX2.getData(), DELTA);
        }
        {
            Matrix<N1, N1> u = VecBuilder.fill(-0.5);
            double dt = 1;
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, u, dt);
            // x = x0 + v t + 0.5 u t ^ 2 = 1 - 0.5 = 0.5
            // v = v0 + u = 1 - 1 = 0
            assertArrayEquals(new double[] { 0.75, 0.5 }, maxX2.getData(), DELTA);
        }
        {
            Matrix<N1, N1> u = VecBuilder.fill(0);
            double dt = 1;
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, u, dt);
            // x = x0 + v t = 1
            // v = v0 = 1
            assertArrayEquals(new double[] { 1, 1 }, maxX2.getData(), DELTA);
        }
        {
            Matrix<N1, N1> u = VecBuilder.fill(0.5);
            double dt = 1;
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, u, dt);
            // x = x0 + v t + 0.5 u t ^ 2 = 1 + 0.5 = 1.5
            // v = v0 + u = 1 + 1 = 2
            assertArrayEquals(new double[] { 1.25, 1.5 }, maxX2.getData(), DELTA);
        }
        {
            Matrix<N1, N1> u = VecBuilder.fill(1);
            double dt = 1;
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, u, dt);
            // x = x0 + v t + 0.5 u t ^ 2 = 1 + 0.5 = 1.5
            // v = v0 + u = 1 + 1 = 2
            assertArrayEquals(new double[] { 1.5, 2 }, maxX2.getData(), DELTA);
        }
        {
            // in the middle somewhere
            Matrix<N1, N1> u = VecBuilder.fill(0.5);
            double dt = 0.5;
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, u, dt);
            // x = x0 + v t + 0.5 u t ^ 2 = 1 + 0.5 = 1.5
            // v = v0 + u = 1 + 1 = 2
            // this is the solution my solver should find below
            assertArrayEquals(new double[] { 0.5625, 1.25 }, maxX2.getData(), DELTA);
        }
    }


    @Test
    void testSolve2d() {
        ShootingSolver<N2, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 1, 20);

        // double-integrator
        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = (x, u) -> {
            return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
        };
        Matrix<N2, N1> x1 = VecBuilder.fill(0, 1);
        {
            if (DEBUG)
                System.out.println("==== end = start");
            Matrix<N2, N1> x2 = VecBuilder.fill(0, 1);
            ShootingSolver<N2, N1>.Solution sol = s.solve(Nat.N2(), Nat.N1(), f, x1, x2, true);
            assertNotNull(sol);
            // u is irrelevant
            assertEquals(0, sol.dt, DELTA);
        }
        {
            if (DEBUG)
                System.out.println("==== end = minU");
            Matrix<N2, N1> goalX2 = NumericalIntegration.rk4(f, x1, VecBuilder.fill(-1), 1);
            if (DEBUG)
                System.out.printf("GOAL: %s\n", goalX2);
            Matrix<N2, N1> x2 = VecBuilder.fill(0.5, 0);
            ShootingSolver<N2, N1>.Solution sol = s.solve(Nat.N2(), Nat.N1(), f, x1, x2, true);
            assertNotNull(sol);
            assertEquals(-1, sol.u.get(0, 0), DELTA);
            assertEquals(1, sol.dt, DELTA);
        }
        {
            if (DEBUG)
                System.out.println("==== end = maxU");
            Matrix<N2, N1> x2 = VecBuilder.fill(1.5, 2);
            ShootingSolver<N2, N1>.Solution sol = s.solve(Nat.N2(), Nat.N1(), f, x1, x2, true);
            assertNotNull(sol);
            assertEquals(1, sol.u.get(0, 0), DELTA);
            assertEquals(1, sol.dt, DELTA);
        }
        {
            if (DEBUG)
                System.out.println("==== end = within the triangle (from above RK4 test)");
            Matrix<N2, N1> x2 = VecBuilder.fill(0.5625, 1.25);
            ShootingSolver<N2, N1>.Solution sol = s.solve(Nat.N2(), Nat.N1(), f, x1, x2, true);
            assertNotNull(sol);
            assertEquals(0.5, sol.u.get(0, 0), DELTA);
            assertEquals(0.5, sol.dt, DELTA);
        }
        {
            if (DEBUG)
                System.out.println("==== end = very infeasible");
            Matrix<N2, N1> x2 = VecBuilder.fill(-10, 10);
            ShootingSolver<N2, N1>.Solution sol = s.solve(Nat.N2(), Nat.N1(), f, x1, x2, true);
            assertNull(sol);
        }
    }

    @Test
    void testNear() {
        ShootingSolver<N2, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 1, 10);
        {
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> x2 = VecBuilder.fill(0, 1);
            assertTrue(s.near(x1, x2));
        }
        {
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> x2 = VecBuilder.fill(0, 1.1);
            assertFalse(s.near(x1, x2));
        }
    }

    @Test
    void testSolve1d() {
        // 1d
        ShootingSolver<N1, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 1, 10);
        // xdot = u
        BiFunction<Matrix<N1, N1>, Matrix<N1, N1>, Matrix<N1, N1>> f = (x, u) -> {
            return VecBuilder.fill(u.get(0, 0));
        };
        Matrix<N1, N1> x1 = VecBuilder.fill(0);
        Matrix<N1, N1> x2 = VecBuilder.fill(0.5);
        Matrix<N1, N1> minU = VecBuilder.fill(-1);
        Matrix<N1, N1> maxU = VecBuilder.fill(1);
        double minDt = 0.99;
        double maxDt = 1.01;
        int index = 0;
        ShootingSolver<N1, N1>.Solution sol = s.solve(Nat.N1(), Nat.N1(), f, x1, x2, minU, maxU, minDt, maxDt, index);
        assertNotNull(sol);
        // since there are 2 free variables there is an infinite number
        // of solutions
        assertEquals(0.5, sol.u.get(0, 0), DELTA);
        assertEquals(1, sol.dt, DELTA);
    }

    @Test
    void testSolve4d() {
        // state is (x, xdot, y, ydot), control is (ux, uy)
        ShootingSolver<N4, N2> s = new ShootingSolver<>(VecBuilder.fill(1, 1), 1, 30);
        // two double-integrators.
        BiFunction<Matrix<N4, N1>, Matrix<N2, N1>, Matrix<N4, N1>> f = (x, u) -> {
            // double x1 = x.get(0, 0);
            double x2 = x.get(1, 0);
            // double y1 = x.get(2, 0);
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
        // check integration
        double dt = 1;
        {
            // motionless
            Matrix<N4, N1> start = VecBuilder.fill(0, 0, 0, 0);
            // input only on x axis.
            Matrix<N2, N1> u = VecBuilder.fill(1, 0);
            Matrix<N4, N1> end = NumericalIntegration.rk4(f, start, u, dt);
            assertArrayEquals(new double[] { 0.5, 1, 0, 0 }, end.getData(), DELTA);
        }
        {
            // motionless
            Matrix<N4, N1> start = VecBuilder.fill(0, 0, 0, 0);
            // same input on both axes.
            Matrix<N2, N1> u = VecBuilder.fill(1, 1);
            Matrix<N4, N1> end = NumericalIntegration.rk4(f, start, u, dt);
            assertArrayEquals(new double[] { 0.5, 1, 0.5, 1 }, end.getData(), DELTA);
        }
        {
            // moving in one axis
            Matrix<N4, N1> start = VecBuilder.fill(0, 0, 0, 1);
            Matrix<N2, N1> u = VecBuilder.fill(1, -1);
            Matrix<N4, N1> end = NumericalIntegration.rk4(f, start, u, dt);
            assertArrayEquals(new double[] { 0.5, 1, 0.5, 0 }, end.getData(), DELTA);
        }
        {
            Matrix<N4, N1> x1 = VecBuilder.fill(0, 1, 0, 0);
            Matrix<N4, N1> x2 = VecBuilder.fill(0, 1, 0, 0);
            ShootingSolver<N4, N2>.Solution sol = s.solve(Nat.N4(), Nat.N2(), f, x1, x2, true);
            assertNotNull(sol);
            // u is irrelevant
            assertEquals(0, sol.dt, DELTA);
        }
        {
            Matrix<N4, N1> x1 = VecBuilder.fill(0, 0, 0, 0);
            Matrix<N4, N1> x2 = VecBuilder.fill(0.5, 1, 0, 0);
            if (DEBUG)
                System.out.printf("===========\nTEST x1 %s x2 %s\n", Util.matStr(x1), Util.matStr(x2));
            ShootingSolver<N4, N2>.Solution sol = s.solve(Nat.N4(), Nat.N2(), f, x1, x2, true);
            assertNotNull(sol);
            assertArrayEquals(new double[] { 1, 0 }, sol.u.getData(), DELTA);
            assertEquals(1, sol.dt, DELTA);
        }
        {
            Matrix<N4, N1> x1 = VecBuilder.fill(0, 0, 0, 0);
            Matrix<N4, N1> x2 = VecBuilder.fill(0.5, 1, -0.5, -1);
            ShootingSolver<N4, N2>.Solution sol = s.solve(Nat.N4(), Nat.N2(), f, x1, x2, true);
            assertNotNull(sol);
            assertArrayEquals(new double[] { 1, -1 }, sol.u.getData(), DELTA);
            assertEquals(1, sol.dt, DELTA);
        }
    }
}
