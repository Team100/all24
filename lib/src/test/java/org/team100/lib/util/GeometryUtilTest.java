package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

class GeometryUtilTest {
    private static final double kDelta = 0.001;

    @Test
    void testSlog() {
        Twist2d twist = GeometryUtil.slog(
                new Pose2d(1, 0, GeometryUtil.kRotationZero));
        assertEquals(1, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0, twist.dtheta, kDelta);

        // this twist represents an arc that ends up at the endpoint.
        twist = GeometryUtil.slog(
                new Pose2d(1, 0, GeometryUtil.kRotation90));
        assertEquals(0.785, twist.dx, kDelta);
        assertEquals(-0.785, twist.dy, kDelta);
        assertEquals(1.571, twist.dtheta, kDelta);
    }

    @Test
    void testDistance() {
        // same pose => 0
        assertEquals(0,
                GeometryUtil.distance(
                        new Pose2d(1, 0, GeometryUtil.kRotationZero),
                        new Pose2d(1, 0, GeometryUtil.kRotationZero)),
                kDelta);
        // 1d distance
        assertEquals(1,
                GeometryUtil.distance(
                        new Pose2d(0, 0, GeometryUtil.kRotationZero),
                        new Pose2d(1, 0, GeometryUtil.kRotationZero)),
                kDelta);
        // 2d distance
        assertEquals(1.414,
                GeometryUtil.distance(
                        new Pose2d(0, 1, GeometryUtil.kRotationZero),
                        new Pose2d(1, 0, GeometryUtil.kRotationZero)),
                kDelta);
        // rotation means a little arc, so the path length is a little longer.
        assertEquals(1.111,
                GeometryUtil.distance(
                        new Pose2d(0, 0, GeometryUtil.kRotationZero),
                        new Pose2d(1, 0, GeometryUtil.kRotation90)),
                kDelta);
        // the arc in this case is the entire quarter circle
        assertEquals(1.571,
                GeometryUtil.distance(
                        new Pose2d(0, 1, GeometryUtil.kRotationZero),
                        new Pose2d(1, 0, GeometryUtil.kRotation90)),
                kDelta);
        // order doesn't matter
        assertEquals(1.571,
                GeometryUtil.distance(
                        new Pose2d(1, 0, GeometryUtil.kRotation90),
                        new Pose2d(0, 1, GeometryUtil.kRotationZero)),
                kDelta);

    }

    @Test
    void testCollinear() {
        assertTrue(GeometryUtil.isColinear(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d())));
        assertFalse(GeometryUtil.isColinear(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d(Math.PI))));
        assertTrue(GeometryUtil.isColinear(
                new Pose2d(0, 0, new Rotation2d(Math.PI / 2)),
                new Pose2d(0, 1, new Rotation2d(Math.PI / 2))));
        assertFalse(GeometryUtil.isColinear(
                new Pose2d(),
                new Pose2d(1, 1, new Rotation2d())));
    }

    @Test
    void testParallel() {
        assertTrue(GeometryUtil.isParallel(new Rotation2d(), new Rotation2d()));
        assertTrue(GeometryUtil.isParallel(new Rotation2d(), new Rotation2d(Math.PI)));

    }

}
