package org.team100.lib.spline;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
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
    void testBounds() {
        {
            // allow 0 degrees
            Pose2d p0 = new Pose2d(0, 0, GeometryUtil.kRotationZero);
            Pose2d p1 = new Pose2d(1, 0, GeometryUtil.kRotationZero);
            assertDoesNotThrow(() -> HolonomicSpline.checkBounds(p0, p1));
        }
        {
            // allow 90 degrees
            Pose2d p0 = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
            Pose2d p1 = new Pose2d(1, 0, GeometryUtil.kRotationZero);
            assertDoesNotThrow(() -> HolonomicSpline.checkBounds(p0, p1));
        }
        {
            // disallow 135 degrees
            Pose2d p0 = new Pose2d(0, 0, new Rotation2d(3 * Math.PI / 4));
            Pose2d p1 = new Pose2d(1, 0, GeometryUtil.kRotationZero);
            assertThrows(IllegalArgumentException.class, () -> HolonomicSpline.checkBounds(p0, p1));
        }
        {
            // disallow u-turn; these are never what you want.
            Pose2d p0 = new Pose2d(0, 0, new Rotation2d(Math.PI));
            Pose2d p1 = new Pose2d(1, 0, GeometryUtil.kRotationZero);
            assertThrows(IllegalArgumentException.class, () -> HolonomicSpline.checkBounds(p0, p1));
        }
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
