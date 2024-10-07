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
    boolean dump = false;
    private static double kEpsilon = 1e-12;

    @Test
    void test() {
        Pose2d a = new Pose2d(new Translation2d(0, 100), Rotation2d.fromDegrees(270));
        Pose2d b = new Pose2d(new Translation2d(50, 0), Rotation2d.fromDegrees(0));
        Pose2d c = new Pose2d(new Translation2d(100, 100), Rotation2d.fromDegrees(90));

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(a, b, new Rotation2d(), new Rotation2d()));
        splines.add(new HolonomicSpline(b, c, new Rotation2d(), new Rotation2d()));

        assertTrue(HolonomicSpline.optimizeSpline(splines) < 0.014);

        Pose2d d = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90));
        Pose2d e = new Pose2d(new Translation2d(0, 50), Rotation2d.fromDegrees(0));
        Pose2d f = new Pose2d(new Translation2d(100, 50), Rotation2d.fromDegrees(-90));
        Pose2d g = new Pose2d(new Translation2d(100, 0), Rotation2d.fromDegrees(-180));

        List<HolonomicSpline> splines1 = new ArrayList<>();
        splines1.add(new HolonomicSpline(d, e, new Rotation2d(), new Rotation2d()));
        splines1.add(new HolonomicSpline(e, f, new Rotation2d(), new Rotation2d()));
        splines1.add(new HolonomicSpline(f, g, new Rotation2d(), new Rotation2d()));

        assertEquals(0.54, HolonomicSpline.optimizeSpline(splines1), 0.01);

        Pose2d h = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
        Pose2d i = new Pose2d(new Translation2d(50, 0), Rotation2d.fromDegrees(0));
        Pose2d j = new Pose2d(new Translation2d(100, 50), Rotation2d.fromDegrees(45));
        Pose2d k = new Pose2d(new Translation2d(150, 0), Rotation2d.fromDegrees(270));
        Pose2d l = new Pose2d(new Translation2d(150, -50), Rotation2d.fromDegrees(270));

        List<HolonomicSpline> splines2 = new ArrayList<>();
        splines2.add(new HolonomicSpline(h, i, new Rotation2d(), new Rotation2d()));
        splines2.add(new HolonomicSpline(i, j, new Rotation2d(), new Rotation2d()));
        splines2.add(new HolonomicSpline(j, k, new Rotation2d(), new Rotation2d()));
        splines2.add(new HolonomicSpline(k, l, new Rotation2d(), new Rotation2d()));

        assertTrue(HolonomicSpline.optimizeSpline(splines2) < 0.05);
        assertEquals(0.0, splines2.get(0).getCurvature(1.0), kEpsilon);
        assertEquals(0.0, splines2.get(2).getCurvature(1.0), kEpsilon);
    }

    @Test
    void testHolonomic() {
        Pose2d a = new Pose2d(new Translation2d(0, 100), Rotation2d.fromDegrees(270));
        Pose2d b = new Pose2d(new Translation2d(50, 0), Rotation2d.fromDegrees(0));
        Pose2d c = new Pose2d(new Translation2d(100, 100), Rotation2d.fromDegrees(90));
        Rotation2d r0 = new Rotation2d();
        Rotation2d r1 = new Rotation2d(Math.PI / 2);
        Rotation2d r2 = new Rotation2d(Math.PI);

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(a, b, r0, r1));
        splines.add(new HolonomicSpline(b, c, r1, r2));

        assertTrue(HolonomicSpline.optimizeSpline(splines) < 0.014);

        Pose2d d = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90));
        Pose2d e = new Pose2d(new Translation2d(0, 50), Rotation2d.fromDegrees(0));
        Pose2d f = new Pose2d(new Translation2d(100, 50), Rotation2d.fromDegrees(-90));
        Pose2d g = new Pose2d(new Translation2d(100, 0), Rotation2d.fromDegrees(-180));

        List<HolonomicSpline> splines1 = new ArrayList<>();
        splines1.add(new HolonomicSpline(d, e, r0, r1));
        splines1.add(new HolonomicSpline(e, f, r1, r2));
        splines1.add(new HolonomicSpline(f, g, r0, r2));

        assertEquals(0.54, HolonomicSpline.optimizeSpline(splines1), 0.01);

        Pose2d h = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
        Pose2d i = new Pose2d(new Translation2d(50, 0), Rotation2d.fromDegrees(0));
        Pose2d j = new Pose2d(new Translation2d(100, 50), Rotation2d.fromDegrees(45));
        Pose2d k = new Pose2d(new Translation2d(150, 0), Rotation2d.fromDegrees(270));
        Pose2d l = new Pose2d(new Translation2d(150, -50), Rotation2d.fromDegrees(270));

        List<HolonomicSpline> splines2 = new ArrayList<>();
        splines2.add(new HolonomicSpline(h, i, r0, r1));
        splines2.add(new HolonomicSpline(i, j, r1, r2));
        splines2.add(new HolonomicSpline(j, k, r0, r2));
        splines2.add(new HolonomicSpline(k, l, r2, r0));

        assertTrue(HolonomicSpline.optimizeSpline(splines2) < 0.05);
        assertEquals(0.0, splines2.get(0).getCurvature(1.0), kEpsilon);
        assertEquals(0.0, splines2.get(2).getCurvature(1.0), kEpsilon);

    }

}
