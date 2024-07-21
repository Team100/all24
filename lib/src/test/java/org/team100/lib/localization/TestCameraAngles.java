package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

class TestCameraAngles {
    private static final double kDelta = 0.001;

    @Test
    void testCameraAngles1() {
        // camera down 30
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(30), 0));
        // sight on-bore
        Rotation3d sight = new Rotation3d();
        Translation2d t = TargetLocalizer.sightToRobotRelative(camera, sight).get();
        // 30-60-90 triangle, so sqrt(3) in x
        assertEquals(1.732, t.getX(), kDelta);
        assertEquals(0, t.getY(), kDelta);
    }

    @Test
    void testCameraAngles2() {
        // camera down 30
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(30), 0));
        // sight 25 down, 10 right
        Rotation3d sight = new Rotation3d(0, Math.toRadians(25), Math.toRadians(-10));
        Translation2d t = TargetLocalizer.sightToRobotRelative(camera, sight).get();
        // steeper than 45 down
        assertEquals(0.691, t.getX(), kDelta);
        // a bit to the right
        assertEquals(-0.194, t.getY(), kDelta);
    }

    @Test
    void testCameraAngles3() {
        // camera down 45
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(45), 0));
        // sight on bore
        Rotation3d sight = new Rotation3d();
        Translation2d t = TargetLocalizer.sightToRobotRelative(camera, sight).get();
        // 45 down means distance = height
        assertEquals(1, t.getX(), kDelta);
        assertEquals(0, t.getY(), kDelta);
    }

    @Test
    void testCameraAngles4() {
        // camera offset in all axes, down 30
        Transform3d camera = new Transform3d(1, 1, 1, new Rotation3d(0, Math.toRadians(30), 0));
        // sight on bore
        Rotation3d sight = new Rotation3d();
        Transform3d t = TargetLocalizer.sightInRobotCoords(camera, sight);
        // offset, sqrt(3) + 1
        // offset should be the same.
        assertEquals(1, t.getX(), kDelta);
        assertEquals(1, t.getY(), kDelta);
        assertEquals(1, t.getZ(), kDelta);
        assertEquals(0, t.getRotation().getX(), kDelta);
        assertEquals(0.523, t.getRotation().getY(), kDelta);
        assertEquals(0, t.getRotation().getZ(), kDelta);
    }

    @Test
    void testCameraAngles5() {
        // camera offset in all axes, down 30
        Transform3d camera = new Transform3d(1, 1, 1, new Rotation3d(0, Math.toRadians(30), 0));
        // sight on bore
        Rotation3d sight = new Rotation3d();
        Translation2d t = TargetLocalizer.sightToRobotRelative(camera, sight).get();
        // offset, sqrt(3) + 1
        assertEquals(2.732, t.getX(), kDelta);
        assertEquals(1, t.getY(), kDelta);
    }

    @Test
    void testCameraAngles6() {
        // down 45, yaw 180
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(45), Math.PI));
        // sight on bore
        Rotation3d sight = new Rotation3d();
        Translation2d t = TargetLocalizer.sightToRobotRelative(camera, sight).get();
        // facing back
        assertEquals(-1, t.getX(), kDelta);
        assertEquals(0, t.getY(), kDelta);
    }

    @Test
    void testCameraAngles7() {
        // down 45, left 90
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(45), Math.PI / 2));
        Rotation3d sight = new Rotation3d();
        Translation2d t = TargetLocalizer.sightToRobotRelative(camera, sight).get();
        // facing left
        assertEquals(0, t.getX(), kDelta);
        assertEquals(1, t.getY(), kDelta);
    }

        @Test
    void testCameraAngles8() {
        // roll inverted, down 45
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(Math.PI, Math.toRadians(45), 0));
        // sight is 45 to the left
        Rotation3d sight = new Rotation3d(0, 0, Math.toRadians(45));
        Translation2d t = TargetLocalizer.sightToRobotRelative(camera, sight).get();
        // camera still pointing at x=1
        assertEquals(1, t.getX(), kDelta);
        // left sight is to the right due to inversion
        assertEquals(-1.414, t.getY(), kDelta);
    }

    @Test
    void testCameraAngles9() {
        // camera down 45
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(45), 0));
        // sight is 45 to the left
        Rotation3d sight = new Rotation3d(0, 0, Math.toRadians(45));
        Translation2d t = TargetLocalizer.sightToRobotRelative(camera, sight).get();
        // camera still pointing at x=1
        assertEquals(1, t.getX(), kDelta);
        // distance to the floor is sqrt(2), so y sight at 45 is the same
        assertEquals(1.414, t.getY(), kDelta);
    }

    @Test
    void testCameraAngles10() {
        // roll 90 right, down 45
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(Math.PI / 2, Math.toRadians(45), 0));
        // sight is down 45
        Rotation3d sight = new Rotation3d(0, Math.toRadians(45), 0);
        Translation2d t = TargetLocalizer.sightToRobotRelative(camera, sight).get();
        // this was wrong before; roll to the right and look "down" -- you're looking to
        // the left.
        // assertEquals(-1, t.getY(), kDelta);
        assertEquals(1, t.getX(), kDelta);
        assertEquals(1.414, t.getY(), kDelta);
    }
}