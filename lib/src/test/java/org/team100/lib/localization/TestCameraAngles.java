package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

import org.team100.lib.motion.drivetrain.Fixtured;

class TestCameraAngles extends Fixtured {
    @Test
    void testCameraAngles1() {
        {
            Transform3d e = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(30), 0));
            Translation2d testTranslation1 = PoseEstimationHelper.cameraRotationToRobotRelative(e, new Rotation3d());
            Translation2d testTranslation2 = PoseEstimationHelper.cameraRotationToRobotRelative(e,
                    new Rotation3d(0, Math.toRadians(25), Math.toRadians(-10)));
            assertEquals(0, testTranslation1.getY(), 0.0001);
            assertEquals(0.7, testTranslation2.getX(), 0.001);
            assertEquals(-0.123465481082, testTranslation2.getY(), 0.001);
            // assertEquals(416,camera.getInverseY(1,0),0.001);
            // assertEquals(123.2,camera.getInverseX(1),0.001);
        }
    }

    @Test
    void testCameraAngles2() {
        {
            Transform3d e = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(45), 0));
            Translation2d testTranslation1 = PoseEstimationHelper.cameraRotationToRobotRelative(e, new Rotation3d());
            assertEquals(1, testTranslation1.getX(), 0.0001);
        }
    }

    @Test
    void testCameraAngles3() {
        {
            Transform3d e = new Transform3d(0, 0, 1, new Rotation3d(0, Math.toRadians(30), 0));
            Translation2d testTranslation1 = PoseEstimationHelper.cameraRotationToRobotRelative(e, new Rotation3d());
            assertEquals(Math.sqrt(3), testTranslation1.getX(), 0.0001);
        }
    }

    @Test
    void testCameraAngles4() {
        {
            Transform3d e = new Transform3d(1, 1, 1, new Rotation3d(0, Math.toRadians(30), 0));
            Translation2d testTranslation1 = PoseEstimationHelper.cameraRotationToRobotRelative(e, new Rotation3d());
            assertEquals(Math.sqrt(3) + 1, testTranslation1.getX(), 0.0001);
            assertEquals(1, testTranslation1.getY(), 0.0001);
        }
    }
}