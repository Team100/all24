package org.team100.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class HolonomicSplineTest {
    private static final double kDelta = 0.001;

    @Test
    void testStationary() {
        HolonomicSpline s = new HolonomicSpline(
                new Pose2d(),
                new Pose2d(),
                new Rotation2d(),
                new Rotation2d());
        Translation2d t = s.getPoint(0);
        assertEquals(0, t.getX(), kDelta);
        t = s.getPoint(1);
        assertEquals(0, t.getX(), kDelta);
        Pose2dWithMotion p = s.getPose2dWithMotion(0);
        assertEquals(0, p.getPose().getX(), kDelta);
        assertEquals(0, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
        p = s.getPose2dWithMotion(1);
        assertEquals(0, p.getPose().getX(), kDelta);
        assertEquals(0, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
    }

    @Test
    void testLinear() {
        HolonomicSpline s = new HolonomicSpline(
                new Pose2d(),
                new Pose2d(1, 0, new Rotation2d()),
                new Rotation2d(),
                new Rotation2d());
        Translation2d t = s.getPoint(0);
        assertEquals(0, t.getX(), kDelta);
        t = s.getPoint(1);
        assertEquals(1, t.getX(), kDelta);
        Pose2dWithMotion p = s.getPose2dWithMotion(0);
        assertEquals(0, p.getPose().getX(), kDelta);
        assertEquals(0, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
        p = s.getPose2dWithMotion(1);
        assertEquals(1, p.getPose().getX(), kDelta);
        assertEquals(0, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
    }

    @Test
    void testLinear2() {
        HolonomicSpline s = new HolonomicSpline(
                new Pose2d(),
                new Pose2d(2, 0, new Rotation2d()),
                new Rotation2d(),
                new Rotation2d());
        Translation2d t = s.getPoint(0);
        assertEquals(0, t.getX(), kDelta);
        t = s.getPoint(1);
        assertEquals(2, t.getX(), kDelta);
        Pose2dWithMotion p = s.getPose2dWithMotion(0);
        assertEquals(0, p.getPose().getX(), kDelta);
        assertEquals(0, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
        p = s.getPose2dWithMotion(1);
        assertEquals(2, p.getPose().getX(), kDelta);
        assertEquals(0, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
    }

    @Test
    void testRotation() {
        HolonomicSpline s = new HolonomicSpline(
                new Pose2d(),
                new Pose2d(),
                new Rotation2d(),
                new Rotation2d(1));
        Translation2d t = s.getPoint(0);
        assertEquals(0, t.getX(), kDelta);
        t = s.getPoint(1);
        assertEquals(0, t.getX(), kDelta);
        Pose2dWithMotion p = s.getPose2dWithMotion(0);
        assertEquals(0, p.getPose().getX(), kDelta);
        assertEquals(0, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
        p = s.getPose2dWithMotion(1);
        assertEquals(0, p.getPose().getX(), kDelta);
        assertEquals(1, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
        p = s.getPose2dWithMotion(0.5);
        assertEquals(0, p.getPose().getX(), kDelta);
        assertEquals(0.5, p.getPose().getRotation().getRadians(), kDelta);
        // so what does this mean?
        assertEquals(1.875, p.getHeadingRate(), kDelta);
    }

}
