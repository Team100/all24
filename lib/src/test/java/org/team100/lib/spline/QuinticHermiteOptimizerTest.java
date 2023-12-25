package org.team100.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class QuinticHermiteOptimizerTest {
    private static double kEpsilon = 1e-12;

    @Test
    void test() {
        Pose2d a = new Pose2d(new Translation2d(0, 100), Rotation2d.fromDegrees(270));
        Pose2d b = new Pose2d(new Translation2d(50, 0), Rotation2d.fromDegrees(0));
        Pose2d c = new Pose2d(new Translation2d(100, 100), Rotation2d.fromDegrees(90));

        List<QuinticHermitePoseSplineNonholonomic> splines = new ArrayList<>();
        splines.add(new QuinticHermitePoseSplineNonholonomic(a, b));
        splines.add(new QuinticHermitePoseSplineNonholonomic(b, c));

        // long startTime = System.currentTimeMillis();
        assertTrue(QuinticHermitePoseSplineNonholonomic.optimizeSpline(splines) < 0.014);
        // System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));

        Pose2d d = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90));
        Pose2d e = new Pose2d(new Translation2d(0, 50), Rotation2d.fromDegrees(0));
        Pose2d f = new Pose2d(new Translation2d(100, 0), Rotation2d.fromDegrees(90));
        Pose2d g = new Pose2d(new Translation2d(100, 100), Rotation2d.fromDegrees(0));

        List<QuinticHermitePoseSplineNonholonomic> splines1 = new ArrayList<>();
        splines1.add(new QuinticHermitePoseSplineNonholonomic(d, e));
        splines1.add(new QuinticHermitePoseSplineNonholonomic(e, f));
        splines1.add(new QuinticHermitePoseSplineNonholonomic(f, g));

        // startTime = System.currentTimeMillis();
        assertTrue(QuinticHermitePoseSplineNonholonomic.optimizeSpline(splines1) < 0.16);
        // System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));


        Pose2d h = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
        Pose2d i = new Pose2d(new Translation2d(50, 0), Rotation2d.fromDegrees(0));
        Pose2d j = new Pose2d(new Translation2d(100, 50), Rotation2d.fromDegrees(45));
        Pose2d k = new Pose2d(new Translation2d(150, 0), Rotation2d.fromDegrees(270));
        Pose2d l = new Pose2d(new Translation2d(150, -50), Rotation2d.fromDegrees(270));

        List<QuinticHermitePoseSplineNonholonomic> splines2 = new ArrayList<>();
        splines2.add(new QuinticHermitePoseSplineNonholonomic(h, i));
        splines2.add(new QuinticHermitePoseSplineNonholonomic(i, j));
        splines2.add(new QuinticHermitePoseSplineNonholonomic(j, k));
        splines2.add(new QuinticHermitePoseSplineNonholonomic(k, l));

        // startTime = System.currentTimeMillis();
        assertTrue(QuinticHermitePoseSplineNonholonomic.optimizeSpline(splines2) < 0.05);
        assertEquals(0.0, splines2.get(0).getCurvature(1.0), kEpsilon);
        assertEquals(0.0, splines2.get(2).getCurvature(1.0), kEpsilon);
        // System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));
    }


    @Test
    void testHolonomic() {
        Pose2d a = new Pose2d(new Translation2d(0, 100), Rotation2d.fromDegrees(270));
        Pose2d b = new Pose2d(new Translation2d(50, 0), Rotation2d.fromDegrees(0));
        Pose2d c = new Pose2d(new Translation2d(100, 100), Rotation2d.fromDegrees(90));
        Rotation2d r0 = new Rotation2d();
        Rotation2d r1 = new Rotation2d(Math.PI/2);
        Rotation2d r2 = new Rotation2d(Math.PI);

        List<QuinticHermitePoseSplineNonholonomic> splines = new ArrayList<>();
        splines.add(new QuinticHermitePoseSplineHolonomic(a, b, r0, r1));
        splines.add(new QuinticHermitePoseSplineHolonomic(b, c, r1, r2));

        // long startTime = System.currentTimeMillis();
        assertTrue(QuinticHermitePoseSplineNonholonomic.optimizeSpline(splines) < 0.014);
        // System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));

        Pose2d d = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90));
        Pose2d e = new Pose2d(new Translation2d(0, 50), Rotation2d.fromDegrees(0));
        Pose2d f = new Pose2d(new Translation2d(100, 0), Rotation2d.fromDegrees(90));
        Pose2d g = new Pose2d(new Translation2d(100, 100), Rotation2d.fromDegrees(0));

        List<QuinticHermitePoseSplineNonholonomic> splines1 = new ArrayList<>();
        splines1.add(new QuinticHermitePoseSplineHolonomic(d, e, r0, r1));
        splines1.add(new QuinticHermitePoseSplineHolonomic(e, f, r1, r2));
        splines1.add(new QuinticHermitePoseSplineHolonomic(f, g, r0, r2));

        // startTime = System.currentTimeMillis();
        assertTrue(QuinticHermitePoseSplineNonholonomic.optimizeSpline(splines1) < 0.16);
        // System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));

        Pose2d h = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
        Pose2d i = new Pose2d(new Translation2d(50, 0), Rotation2d.fromDegrees(0));
        Pose2d j = new Pose2d(new Translation2d(100, 50), Rotation2d.fromDegrees(45));
        Pose2d k = new Pose2d(new Translation2d(150, 0), Rotation2d.fromDegrees(270));
        Pose2d l = new Pose2d(new Translation2d(150, -50), Rotation2d.fromDegrees(270));

        List<QuinticHermitePoseSplineNonholonomic> splines2 = new ArrayList<>();
        splines2.add(new QuinticHermitePoseSplineHolonomic(h, i, r0, r1));
        splines2.add(new QuinticHermitePoseSplineHolonomic(i, j, r1, r2));
        splines2.add(new QuinticHermitePoseSplineHolonomic(j, k, r0, r2));
        splines2.add(new QuinticHermitePoseSplineHolonomic(k, l, r2, r0));

        // startTime = System.currentTimeMillis();
        assertTrue(QuinticHermitePoseSplineNonholonomic.optimizeSpline(splines2) < 0.05);
        assertEquals(0.0, splines2.get(0).getCurvature(1.0), kEpsilon);
        assertEquals(0.0, splines2.get(2).getCurvature(1.0), kEpsilon);
        // System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));
    }

}
