package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

class OverwatchHelperTest {
    private static final double kDelta = 0.001;

    @Test
    void testAboveTheOrigin() {
        // camera 3m above the origin corner, pitched down
        Transform3d cameraInFieldCoordinates = new Transform3d(
                new Translation3d(0, 0, 3),
                new Rotation3d(0, Math.PI / 2, 0));
        // tag on the floor at the origin
        // 3 meters ahead at the origin
        Transform3d tagInCameraCoordinates = new Transform3d(
                new Translation3d(3, 0, 0),
                new Rotation3d(0, 0, 0));

        Transform3d result = OverwatchHelper.tagInFieldCoordinates(cameraInFieldCoordinates, tagInCameraCoordinates);
        assertEquals(0, result.getTranslation().getX(), kDelta);
        assertEquals(0, result.getTranslation().getY(), kDelta);
        assertEquals(0, result.getTranslation().getZ(), kDelta);
        assertEquals(0, result.getRotation().getX(), kDelta);
        // pitch down == tag is facing up
        assertEquals(Math.PI / 2, result.getRotation().getY(), kDelta);
        assertEquals(0, result.getRotation().getZ(), kDelta);

        Pose2d pose = OverwatchHelper.toPose(result);
        assertEquals(0, pose.getX(), kDelta);
        assertEquals(0, pose.getY(), kDelta);
        assertEquals(0, pose.getRotation().getRadians(), kDelta);
    }

    @Test
    void testAboveTheSide() {
        // looking down diagonally from the railing, on the side
        Transform3d cameraInFieldCoordinates = new Transform3d(
                new Translation3d(8, 10, 4),
                new Rotation3d(0, Math.PI / 4, -Math.PI / 2));
        // tag in the center of the field
        // 4*sqrt(2)=5.657
        Transform3d tagInCameraCoordinates = new Transform3d(
                new Translation3d(5.657, 0, 0),
                new Rotation3d(0, Math.PI / 4, 0));

        Transform3d result = OverwatchHelper.tagInFieldCoordinates(cameraInFieldCoordinates, tagInCameraCoordinates);
        assertEquals(8, result.getTranslation().getX(), kDelta);
        // 10-4=6
        assertEquals(6, result.getTranslation().getY(), kDelta);
        assertEquals(0, result.getTranslation().getZ(), kDelta);
        assertEquals(0, result.getRotation().getX(), kDelta);
        // pitch down == tag is facing up
        assertEquals(Math.PI / 2, result.getRotation().getY(), kDelta);
        // rotated
        assertEquals(-Math.PI / 2, result.getRotation().getZ(), kDelta);

        Pose2d pose = OverwatchHelper.toPose(result);
        assertEquals(8, pose.getX(), kDelta);
        assertEquals(6, pose.getY(), kDelta);
        assertEquals(-Math.PI / 2, pose.getRotation().getRadians(), kDelta);
    }

    @Test
    void testFromTheEnd() {
        // camera 2m above the origin corner, looking straight ahead
        Transform3d cameraInFieldCoordinates = new Transform3d(
                new Translation3d(0, 0, 2),
                new Rotation3d(0, 0, 0));
        // tag right in front of the camera, 3 meters away

        Transform3d tagInCameraCoordinates = new Transform3d(
                new Translation3d(3, 0, 0),
                new Rotation3d(0, 0, 0));

        Transform3d result = OverwatchHelper.tagInFieldCoordinates(cameraInFieldCoordinates, tagInCameraCoordinates);
        // 3 meters away
        assertEquals(3, result.getTranslation().getX(), kDelta);
        assertEquals(0, result.getTranslation().getY(), kDelta);
        // 2 meters up
        assertEquals(2, result.getTranslation().getZ(), kDelta);
        // not rotated
        assertEquals(0, result.getRotation().getX(), kDelta);
        assertEquals(0, result.getRotation().getY(), kDelta);
        assertEquals(0, result.getRotation().getZ(), kDelta);

        Pose2d pose = OverwatchHelper.toPose(result);
        assertEquals(3, pose.getX(), kDelta);
        assertEquals(0, pose.getY(), kDelta);
        assertEquals(0, pose.getRotation().getRadians(), kDelta);
    }

    @Test
    void testFromTheSide1() {
        // camera 2m above the floor, off to the left side, looking towards the middle.
        Transform3d cameraInFieldCoordinates = new Transform3d(
                new Translation3d(8, 10, 2),
                new Rotation3d(0, 0, -Math.PI / 2));
        // tag right in front of the camera, 3 meters away
        Transform3d tagInCameraCoordinates = new Transform3d(
                new Translation3d(3, 0, 0),
                new Rotation3d(0, 0, 0));
        Transform3d result = OverwatchHelper.tagInFieldCoordinates(cameraInFieldCoordinates, tagInCameraCoordinates);
        assertEquals(8, result.getTranslation().getX(), kDelta);
        assertEquals(7, result.getTranslation().getY(), kDelta);
        // 2 meters up
        assertEquals(2, result.getTranslation().getZ(), kDelta);
        // not rotated
        assertEquals(0, result.getRotation().getX(), kDelta);
        assertEquals(0, result.getRotation().getY(), kDelta);
        assertEquals(-Math.PI / 2, result.getRotation().getZ(), kDelta);

        Pose2d pose = OverwatchHelper.toPose(result);
        assertEquals(8, pose.getX(), kDelta);
        assertEquals(7, pose.getY(), kDelta);
        assertEquals(-Math.PI / 2, pose.getRotation().getRadians(), kDelta);
    }

    @Test
    void testFromTheSide2() {
        // camera 2m above the floor, off to the left side, looking towards the middle.
        Transform3d cameraInFieldCoordinates = new Transform3d(
                new Translation3d(8, 10, 2),
                new Rotation3d(0, 0, -Math.PI / 2));
        // tag to the right
        Transform3d tagInCameraCoordinates = new Transform3d(
                new Translation3d(3, -1, 0),
                new Rotation3d(0, 0, 0));
        Transform3d result = OverwatchHelper.tagInFieldCoordinates(cameraInFieldCoordinates, tagInCameraCoordinates);
        // a bit less x than case 1
        assertEquals(7, result.getTranslation().getX(), kDelta);
        assertEquals(7, result.getTranslation().getY(), kDelta);
        // 2 meters up
        assertEquals(2, result.getTranslation().getZ(), kDelta);
        // not rotated
        assertEquals(0, result.getRotation().getX(), kDelta);
        assertEquals(0, result.getRotation().getY(), kDelta);
        assertEquals(-Math.PI / 2, result.getRotation().getZ(), kDelta);

        Pose2d pose = OverwatchHelper.toPose(result);
        assertEquals(7, pose.getX(), kDelta);
        assertEquals(7, pose.getY(), kDelta);
        assertEquals(-Math.PI / 2, pose.getRotation().getRadians(), kDelta);
    }
}
