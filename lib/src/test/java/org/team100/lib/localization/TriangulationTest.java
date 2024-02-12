package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * In 2024 there are multiple tags at the speaker target, so we can triangulate
 * for more accuracy.
 * 
 * https://en.wikipedia.org/wiki/Triangulation_(surveying)
 */
class TriangulationTest {
    private static final double kDelta = 0.001;

    @Test
    void testSimple() {
        // robot true position is (0,0)
        // camera facing (0,0)
        // targets are at (1, -1) and (1, 1)
        // these are the known locations in field coordinates
        Translation2d a = new Translation2d(1, -1);
        Translation2d b = new Translation2d(1, 1);

        // interior angles from baseline to camera
        double alpha = Math.PI / 4;
        double beta = Math.PI / 4;
        double baselineLength = 2;
        double perpendicularDistance = baselineLength * Math.sin(alpha) * Math.sin(beta) / Math.sin(alpha + beta);
        assertEquals(1, perpendicularDistance, kDelta);
        double alphaL = perpendicularDistance / Math.tan(alpha);
        assertEquals(1, alphaL, kDelta);
        double betaL = perpendicularDistance / Math.tan(beta);
        assertEquals(1, betaL, kDelta);
        // magically know that alpha is to the left
        Translation2d toAlpha = new Translation2d(perpendicularDistance, -alphaL);
        Translation2d toBeta = new Translation2d(perpendicularDistance, betaL);
        // using each point to estimate the robot location
        Translation2d alphaT = a.minus(toAlpha);
        Translation2d betaT = b.minus(toBeta);
        assertEquals(0, alphaT.getX(), kDelta);
        assertEquals(0, alphaT.getY(), kDelta);
        assertEquals(0, betaT.getX(), kDelta);
        assertEquals(0, betaT.getY(), kDelta);
    }

    /**
     * Same as above but using angles from camera point of view, using NWU.
     */
    @Test
    void testSimple2() {
        // robot true position is (0,0)
        // camera facing (0,0)
        // targets are at (1, -1) and (1, 1)
        // these are the known locations in field coordinates
        // remember a is to the right
        Translation2d a = new Translation2d(1, -1);
        // b is to the left
        Translation2d b = new Translation2d(1, 1);

        // angles from camera-zero (x forward)
        // a is to the right so this is negative rotation
        double theta = -1.0 * Math.PI / 4;
        // b is to the left so positive rotation
        double phi = Math.PI / 4;
        double baselineLength = 2;
        // magically know which one to put first in the subtraction
        double perpendicularDistance = baselineLength * Math.cos(theta) * Math.cos(phi) / Math.sin(phi - theta);
        assertEquals(1, perpendicularDistance, kDelta);
        double alphaL = perpendicularDistance * Math.tan(theta);
        double betaL = perpendicularDistance * Math.tan(phi);
        // no more minus sign here
        Translation2d toAlpha = new Translation2d(perpendicularDistance, alphaL);
        Translation2d toBeta = new Translation2d(perpendicularDistance, betaL);
        // using each point to estimate the robot location
        Translation2d alphaT = a.minus(toAlpha);
        Translation2d betaT = b.minus(toBeta);
        assertEquals(0, alphaT.getX(), kDelta);
        assertEquals(0, alphaT.getY(), kDelta);
        assertEquals(0, betaT.getX(), kDelta);
        assertEquals(0, betaT.getY(), kDelta);
    }

    /**
     * Getting rid of magic, it works in either order.
     */
    @Test
    void testSimple3() {
        // robot true position is (0,0)
        // camera facing (0,0)
        // targets are at (1, -1) and (1, 1)
        // these are the known locations in field coordinates
        // remember a is to the right
        Translation2d a = new Translation2d(1, -1);
        // b is to the left
        Translation2d b = new Translation2d(1, 1);
        // angles from camera-zero (x forward)
        // a is to the right so this is negative rotation
        double thetaA = -1.0 * Math.PI / 4;
        // b is to the left so positive rotation
        double thetaB = Math.PI / 4;

        {
            Translation2d estimate = foo(a, b, thetaA, thetaB);
            assertEquals(0, estimate.getX(), kDelta);
            assertEquals(0, estimate.getY(), kDelta);
        }
        {
            Translation2d estimate = foo(b, a, thetaB, thetaA);
            assertEquals(0, estimate.getX(), kDelta);
            assertEquals(0, estimate.getY(), kDelta);
        }
    }

    Translation2d foo(Translation2d a, Translation2d b, double thetaA, double thetaB) {
        // TODO make this work for any orientation of a and b
        double bToA = a.minus(b).getY();
        double perpendicularDistance = bToA * Math.cos(thetaA) * Math.cos(thetaB) / Math.sin(thetaA - thetaB);
        double alphaL = perpendicularDistance * Math.tan(thetaA);
        double betaL = perpendicularDistance * Math.tan(thetaB);
        Translation2d toAlpha = new Translation2d(perpendicularDistance, alphaL);
        Translation2d toBeta = new Translation2d(perpendicularDistance, betaL);
        // these should be the same
        Translation2d alphaT = a.minus(toAlpha);
        Translation2d betaT = b.minus(toBeta);
        return alphaT.plus(betaT).div(2);
    }

    /**
     * The vector algebra way.
     * 
     * For notation see https://apps.dtic.mil/sti/tr/pdf/ADA559309.pdf
     */
    @Test
    void testVectors() {
        Translation2d T1 = new Translation2d(1, 1);
        Translation2d T2 = new Translation2d(1, -1);
        Matrix<N2, N1> t1 = MatBuilder.fill(Nat.N2(), Nat.N1(), T1.getX(), T1.getY());
        Matrix<N2, N1> t2 = MatBuilder.fill(Nat.N2(), Nat.N1(), T2.getX(), T2.getY());
        Translation2d T = T2.minus(T1);
        assertEquals(0, T.getX(), kDelta);
        // from +y to -y, left to right
        assertEquals(-2, T.getY(), kDelta);
        Matrix<N2, N1> t = MatBuilder.fill(Nat.N2(), Nat.N1(), T.getX(), T.getY());

        // unit vectors
        // left side
        Rotation2d r1 = new Rotation2d(-3 * Math.PI / 4);
        // right side
        Rotation2d r2 = new Rotation2d(3 * Math.PI / 4);
        Translation2d d1 = new Translation2d(1, r1);
        assertEquals(-0.707, d1.getX(), kDelta);
        assertEquals(-0.707, d1.getY(), kDelta);
        Translation2d d2 = new Translation2d(1, r2);
        assertEquals(-0.707, d2.getX(), kDelta);
        assertEquals(0.707, d2.getY(), kDelta);

        Matrix<N2, N2> d = MatBuilder.fill(Nat.N2(), Nat.N2(), d1.getX(), -d2.getX(), d1.getY(), -d2.getY());
        Matrix<N2, N1> lambda = d.solve(t);
        assertEquals(Math.sqrt(2), lambda.get(0, 0), kDelta);
        assertEquals(Math.sqrt(2), lambda.get(1, 0), kDelta);

        Matrix<N2, N2> D = MatBuilder.fill(Nat.N2(), Nat.N2(), d1.getX(), d2.getX(), d1.getY(), d2.getY());
        Matrix<N2, N1> X = D.times(lambda).plus(t1).plus(t2).times(0.5);
        Translation2d x = new Translation2d(X.get(0, 0), X.get(1, 0));
        assertEquals(0, x.getX(), kDelta);
        assertEquals(0, x.getY(), kDelta);
    }

    @Test
    void testCameraCentricVectors() {
        // known tag locations
        Translation2d T1 = new Translation2d(1, 1);
        Translation2d T2 = new Translation2d(1, -1);
        Matrix<N2, N1> t1 = MatBuilder.fill(Nat.N2(), Nat.N1(), T1.getX(), T1.getY());
        Matrix<N2, N1> t2 = MatBuilder.fill(Nat.N2(), Nat.N1(), T2.getX(), T2.getY());
        Translation2d T = T2.minus(T1);
        assertEquals(0, T.getX(), kDelta);
        // from +y to -y, left to right
        assertEquals(-2, T.getY(), kDelta);
        Matrix<N2, N1> t = MatBuilder.fill(Nat.N2(), Nat.N1(), T.getX(), T.getY());

        // unit vectors from the camera point of view
        // inverted compared the case above
        // left side
        Rotation2d r1 = new Rotation2d(Math.PI / 4);
        // right side
        Rotation2d r2 = new Rotation2d(-Math.PI / 4);
        Translation2d d1 = new Translation2d(1, r1);
        assertEquals(0.707, d1.getX(), kDelta);
        assertEquals(0.707, d1.getY(), kDelta);
        Translation2d d2 = new Translation2d(1, r2);
        assertEquals(0.707, d2.getX(), kDelta);
        assertEquals(-0.707, d2.getY(), kDelta);

        // inverted compared the case above
        Matrix<N2, N2> d = MatBuilder.fill(Nat.N2(), Nat.N2(), -d1.getX(), d2.getX(), -d1.getY(), d2.getY());
        Matrix<N2, N1> lambda = d.solve(t);
        assertEquals(Math.sqrt(2), lambda.get(0, 0), kDelta);
        assertEquals(Math.sqrt(2), lambda.get(1, 0), kDelta);

        // invert the d vectors
        Matrix<N2, N2> D = MatBuilder.fill(Nat.N2(), Nat.N2(), -d1.getX(), -d2.getX(), -d1.getY(), -d2.getY());
        Matrix<N2, N1> X = D.times(lambda).plus(t1).plus(t2).times(0.5);
        Translation2d x = new Translation2d(X.get(0, 0), X.get(1, 0));
        assertEquals(0, x.getX(), kDelta);
        assertEquals(0, x.getY(), kDelta);
    }
}
