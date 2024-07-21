package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

class TestCameraAngles {
    private static final double kDelta = 0.0001;

    @Test
    void testCameraAngles1() {
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(30), 0));
        Translation2d testTranslation1 = PoseEstimationHelper
                .cameraRotationToRobotRelative(camera, new Rotation3d()).get();
        assertEquals(Math.sqrt(3), testTranslation1.getX(), kDelta);
        assertEquals(0, testTranslation1.getY(), kDelta);
    }

    @Test
    void testCameraAngles2() {
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(30), 0));
        Translation2d testTranslation2 = PoseEstimationHelper
                .cameraRotationToRobotRelative(
                        camera,
                        new Rotation3d(0, Math.toRadians(25), Math.toRadians(-10)))
                .get();
        assertEquals(0.7002, testTranslation2.getX(), kDelta);
        assertEquals(-0.123465481082, testTranslation2.getY(), kDelta);
    }

    @Test
    void testCameraAngles3() {
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(45), 0));
        Translation2d testTranslation1 = PoseEstimationHelper
                .cameraRotationToRobotRelative(camera, new Rotation3d()).get();
        assertEquals(1, testTranslation1.getX(), kDelta);
        assertEquals(0, testTranslation1.getY(), kDelta);
    }

    @Test
    void testCameraAngles4() {
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(30), 0));
        Translation2d testTranslation1 = PoseEstimationHelper
                .cameraRotationToRobotRelative(camera, new Rotation3d()).get();
        assertEquals(Math.sqrt(3), testTranslation1.getX(), kDelta);
        assertEquals(0, testTranslation1.getY(), kDelta);
    }

    @Test
    void testCameraAngles5() {
        Transform3d camera = new Transform3d(1, 1, 1, new Rotation3d(0, Math.toRadians(30), 0));
        Translation2d testTranslation1 = PoseEstimationHelper
                .cameraRotationToRobotRelative(camera, new Rotation3d()).get();
        assertEquals(Math.sqrt(3) + 1, testTranslation1.getX(), kDelta);
        assertEquals(1, testTranslation1.getY(), kDelta);
    }

    @Test
    void testCameraAngles6() {
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(45), Math.PI));
        Translation2d testTranslation1 = PoseEstimationHelper
                .cameraRotationToRobotRelative(camera, new Rotation3d()).get();
        assertEquals(-1, testTranslation1.getX(), kDelta);
        assertEquals(0, testTranslation1.getY(), kDelta);
    }

    @Test
    void testCameraAngles7() {
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(45), Math.PI / 2));
        Translation2d testTranslation2 = PoseEstimationHelper
                .cameraRotationToRobotRelative(camera, new Rotation3d()).get();
        assertEquals(0, testTranslation2.getX(), kDelta);
        assertEquals(1, testTranslation2.getY(), kDelta);
    }

    @Test
    void testCameraAngles8() {
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(Math.PI, Math.toRadians(45), 0));
        Translation2d testTranslation1 = PoseEstimationHelper
                .cameraRotationToRobotRelative(camera, new Rotation3d(0, 0, Math.toRadians(45))).get();
        assertEquals(1, testTranslation1.getX(), kDelta);
        assertEquals(-1, testTranslation1.getY(), kDelta);
    }

    @Test
    void testCameraAngles9() {
        Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(Math.PI / 2, Math.toRadians(45), 0));
        Translation2d testTranslation2 = PoseEstimationHelper
                .cameraRotationToRobotRelative(camera, new Rotation3d(0, Math.toRadians(45), 0)).get();
        // this was wrong before; roll to the right and look "down" -- you're looking to
        // the left.
        // assertEquals(-1, testTranslation2.getY(), kDelta);
        assertEquals(1, testTranslation2.getX(), kDelta);
        assertEquals(1, testTranslation2.getY(), kDelta);
    }
}