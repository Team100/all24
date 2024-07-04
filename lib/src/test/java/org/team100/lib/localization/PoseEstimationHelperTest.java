package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;

class PoseEstimationHelperTest {
    private static final double kDelta = 0.01;
    Logger m_logger = Telemetry.get().testLogger();

    public PoseEstimationHelperTest() throws IOException {
        // load the JNI
        CameraServerCvJNI.forceLoad();
    }

    @Test
    void testGetRobotPoseInFieldCoords2() {
        // trivial example: if camera offset happens to match the camera global pose
        // then of course the robot global pose is the origin.
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));
        Transform3d tagInCameraCoords = new Transform3d(new Translation3d(1, 0, 0), new Rotation3d());
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        Pose3d robotPoseInFieldCoords = PoseEstimationHelper.applyCameraOffset(
                cameraInFieldCoords,
                cameraInRobotCoords);

        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    void testGetRobotPoseInFieldCoords3() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));
        Translation3d tagTranslationInCameraCoords = new Translation3d(1, 0, 0);
        Rotation3d tagRotationInCameraCoords = new Rotation3d(0, 0, 0);
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        Pose3d robotPoseInFieldCoords = PoseEstimationHelper.applyCameraOffset(
                cameraInFieldCoords,
                cameraInRobotCoords);
        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    void testGetRobotPoseInFieldCoords4() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));

        Rotation3d cameraRotationInFieldCoords = new Rotation3d();

        // one meter range (Z forward)
        // pure tilt note we don't actually use this

        Blip24 blip = new Blip24(7,
                new Transform3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, 0)));

        Translation3d tagTranslationInCameraCoords = PoseEstimationHelper.blipToTranslation(blip);
        Rotation3d tagRotationInCameraCoords = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);

        Pose3d robotPoseInFieldCoords = PoseEstimationHelper.applyCameraOffset(
                cameraInFieldCoords,
                cameraInRobotCoords);
        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    void testGetRobotPoseInFieldCoordsUsingCameraRotation24() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));

        // one meter range (Z forward)
        // identity rotation
        Blip24 blip = new Blip24(5,
                new Transform3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, 0)));

        Pose3d robotPoseInFieldCoords = PoseEstimationHelper.getRobotPoseInFieldCoords(
                cameraInRobotCoords,
                tagInFieldCoords,
                blip);

        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    void testGetRobotPoseInFieldCoords524() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));

        // identity rotation
        // one meter range (Z forward)
        Blip24 blip = new Blip24(5,
                new Transform3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, 0)));

        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d();

        PoseEstimationHelper helper = new PoseEstimationHelper(m_logger);
        Pose3d robotPoseInFieldCoords = helper.getRobotPoseInFieldCoords(
                cameraInRobotCoords,
                tagInFieldCoords,
                blip,
                robotRotationInFieldCoordsFromGyro);

        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    void testCameraRotationInFieldCoords() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(0.5, 0, 0.5), // front camera
                new Rotation3d(0, -Math.PI / 4, Math.PI / 4)); // pi/4 tilt up, pi/4 yaw left
        // robot rotation should only ever be yaw
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 2);
        Rotation3d cameraRotationInFieldCoords = PoseEstimationHelper.cameraRotationInFieldCoords(
                cameraInRobotCoords,
                robotRotationInFieldCoordsFromGyro);
        assertEquals(0, cameraRotationInFieldCoords.getX(), kDelta); // still no roll
        assertEquals(-Math.PI / 4, cameraRotationInFieldCoords.getY(), kDelta); // same tilt
        assertEquals(3 * Math.PI / 4, cameraRotationInFieldCoords.getZ(), kDelta);
    }

    @Test
    void testBlip24ToTransform() {
        { // identity
            Blip24 blip = new Blip24(5,
                    new Transform3d(
                            new Translation3d(),
                            new Rotation3d()));
            Transform3d transform3d = PoseEstimationHelper.blipToTransform(blip);
            assertEquals(0, transform3d.getX(), kDelta);
            assertEquals(0, transform3d.getY(), kDelta);
            assertEquals(0, transform3d.getZ(), kDelta);
            assertEquals(0, transform3d.getRotation().getX(), kDelta);
            assertEquals(0, transform3d.getRotation().getY(), kDelta);
            assertEquals(0, transform3d.getRotation().getZ(), kDelta);
        }
        {
            Blip24 blip = new Blip24(5,
                    new Transform3d(
                            new Translation3d(-2, -1, 3),
                            new Rotation3d(Math.PI / 4, 0, 0)));
            Transform3d transform3d = PoseEstimationHelper.blipToTransform(blip);
            assertEquals(3, transform3d.getX(), kDelta);
            assertEquals(2, transform3d.getY(), kDelta);
            assertEquals(1, transform3d.getZ(), kDelta);
            assertEquals(0, transform3d.getRotation().getX(), kDelta);
            assertEquals(-Math.PI / 4, transform3d.getRotation().getY(), kDelta);
            assertEquals(0, transform3d.getRotation().getZ(), kDelta);
        }
    }

    @Test
    void testBlip24ToTranslation() {
        // Blip is "z-forward", one meter up, two meters left, three meters ahead
        // rotation doesn't matter
        Blip24 blip = new Blip24(5,
                new Transform3d(
                        new Translation3d(-2, -1, 3),
                        new Rotation3d()));

        Translation3d nwuTranslation = PoseEstimationHelper.blipToTranslation(blip);

        // NWU values now
        assertEquals(3, nwuTranslation.getX(), kDelta);
        assertEquals(2, nwuTranslation.getY(), kDelta);
        assertEquals(1, nwuTranslation.getZ(), kDelta);
    }

    @Test
    void testBlip24ToRotation() {
        { // identity rotation
            Blip24 blip = new Blip24(5,
                    new Transform3d(
                            new Translation3d(-2, -1, 3),
                            new Rotation3d()));

            Rotation3d nwuRotation = PoseEstimationHelper.blipToRotation(blip);
            assertEquals(0, nwuRotation.getX(), kDelta);
            assertEquals(0, nwuRotation.getY(), kDelta);
            assertEquals(0, nwuRotation.getZ(), kDelta);
        }
        {
            // one meter range (Z forward)
            // tilt up in camera frame = +x rot
            Blip24 blip = new Blip24(5,
                    new Transform3d(
                            new Translation3d(0, Math.sqrt(2) / 2, Math.sqrt(2) / 2),
                            new Rotation3d(Math.PI / 4, 0, 0)));

            Rotation3d nwuRotation = PoseEstimationHelper.blipToRotation(blip);
            // tilt up in NWU is -y
            assertEquals(0, nwuRotation.getX(), kDelta);
            assertEquals(-Math.PI / 4, nwuRotation.getY(), kDelta);
            assertEquals(0, nwuRotation.getZ(), kDelta);
        }
        {
            // one meter range (Z forward)
            // pan right in camera frame = +y rot
            Blip24 blip = new Blip24(5,
                    new Transform3d(
                            new Translation3d(0, Math.sqrt(2) / 2, Math.sqrt(2) / 2),
                            new Rotation3d(0, Math.PI / 4, 0)));

            Rotation3d nwuRotation = PoseEstimationHelper.blipToRotation(blip);
            // pan right in NWU is -z
            assertEquals(0, nwuRotation.getX(), kDelta);
            assertEquals(0, nwuRotation.getY(), kDelta);
            assertEquals(-Math.PI / 4, nwuRotation.getZ(), kDelta);
        }
    }

    /**
     * We don't trust the camera's estimate of tag rotation. Instead, since we know
     * the pose of the tag, and we know the rotation of the camera (from the
     * gyro/compass and the camera offset), we calculate what the tag rotation
     * *should* be.
     */
    @Test
    void testtagRotationInRobotCoordsFromGyro() {
        {
            Rotation3d tagRotationInFieldCoords = new Rotation3d(0, 0, 0); // on the far wall
            Rotation3d cameraRotationInFieldCoords = new Rotation3d(0, 0, Math.PI / 4); // 45 degrees left
            Rotation3d tagRotationInRobotCoordsFromGyro = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                    tagRotationInFieldCoords,
                    cameraRotationInFieldCoords);
            assertEquals(0, tagRotationInRobotCoordsFromGyro.getX(), kDelta);
            assertEquals(0, tagRotationInRobotCoordsFromGyro.getY(), kDelta);
            assertEquals(-Math.PI / 4, tagRotationInRobotCoordsFromGyro.getZ(), kDelta); // 45 degrees right
        }
        {
            Rotation3d tagRotationInFieldCoords = new Rotation3d(0, 0, 0); // on the far wall
            Rotation3d cameraRotationInFieldCoords = new Rotation3d(0, -Math.PI / 4, 0); // 45 degrees up
            Rotation3d tagRotationInRobotCoordsFromGyro = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                    tagRotationInFieldCoords,
                    cameraRotationInFieldCoords);
            assertEquals(0, tagRotationInRobotCoordsFromGyro.getX(), kDelta);
            assertEquals(Math.PI / 4, tagRotationInRobotCoordsFromGyro.getY(), kDelta); // 45 degrees down
            assertEquals(0, tagRotationInRobotCoordsFromGyro.getZ(), kDelta);
        }
        {
            Rotation3d tagRotationInFieldCoords = new Rotation3d(0, 0, 0); // on the far wall
            Rotation3d cameraRotationInFieldCoords = new Rotation3d(0, -Math.PI / 4, Math.PI / 4); // pan/tilt
            Rotation3d tagRotationInRobotCoordsFromGyro = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                    tagRotationInFieldCoords,
                    cameraRotationInFieldCoords);
            // composing and then inverting multiple rotations yields this:
            assertEquals(-0.615, tagRotationInRobotCoordsFromGyro.getX(), kDelta);
            assertEquals(0.523, tagRotationInRobotCoordsFromGyro.getY(), kDelta);
            assertEquals(-0.955, tagRotationInRobotCoordsFromGyro.getZ(), kDelta);
        }
    }

    @Test
    void testToFieldCoordinates() {
        {
            // opposite corner of 1,1 square
            Transform3d tagInCameraCords = new Transform3d(
                    new Translation3d(Math.sqrt(2), 0, 0),
                    new Rotation3d(0, 0, -Math.PI / 4));
            Pose3d tagInFieldCords = new Pose3d(
                    new Translation3d(1, 1, 0),
                    new Rotation3d());
            Pose3d toFieldCoordinates = PoseEstimationHelper.toFieldCoordinates(
                    tagInCameraCords,
                    tagInFieldCords);
            assertEquals(0, toFieldCoordinates.getX(), kDelta);
            assertEquals(0, toFieldCoordinates.getY(), kDelta);
            assertEquals(0, toFieldCoordinates.getZ(), kDelta);
            assertEquals(0, toFieldCoordinates.getRotation().getX(), kDelta);
            assertEquals(0, toFieldCoordinates.getRotation().getY(), kDelta);
            assertEquals(Math.PI / 4, toFieldCoordinates.getRotation().getZ(), kDelta); // pan45
        }
        {
            // in front, high
            Transform3d tagInCameraCords = new Transform3d(
                    new Translation3d(Math.sqrt(2), 0, 0),
                    new Rotation3d(0, Math.PI / 4, 0));
            Pose3d tagInFieldCords = new Pose3d(
                    new Translation3d(1, 0, 1),
                    new Rotation3d());
            Pose3d toFieldCoordinates = PoseEstimationHelper.toFieldCoordinates(
                    tagInCameraCords,
                    tagInFieldCords);
            assertEquals(0, toFieldCoordinates.getX(), kDelta);
            assertEquals(0, toFieldCoordinates.getY(), kDelta);
            assertEquals(0, toFieldCoordinates.getZ(), kDelta);
            assertEquals(0, toFieldCoordinates.getRotation().getX(), kDelta);
            assertEquals(-Math.PI / 4, toFieldCoordinates.getRotation().getY(), kDelta); // tilt45
            assertEquals(0, toFieldCoordinates.getRotation().getZ(), kDelta);
        }
    }

    @Test
    void testApplyCameraOffset() {
        // trivial example: if camera offset happens to match the camera global pose
        // then of course the robot global pose is the origin.
        Pose3d cameraInFieldCoords = new Pose3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d robotPoseInFieldCoords = PoseEstimationHelper.applyCameraOffset(
                cameraInFieldCoords,
                cameraInRobotCoords);
        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    void testTagRotationIncorrect24() throws IOException {
        // this illustrates the WRONG WRONG WRONG tag orientation.

        // say we're playing blue, on the blue side, looking at tag 7.
        // the robot is facing 180, and the camera returns this.
        // note that the camera code returns the identity rotation when
        // it's looking straight at a tag, which implies "into the page"
        // orientation.

        Blip24 blip = new Blip24(7,
                new Transform3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, 0)));

        Transform3d tagInCameraCoords = PoseEstimationHelper.blipToTransform(blip);
        assertEquals(1, tagInCameraCoords.getX(), kDelta);
        assertEquals(0, tagInCameraCoords.getY(), kDelta);
        assertEquals(0, tagInCameraCoords.getZ(), kDelta);
        assertEquals(0, tagInCameraCoords.getRotation().getX(), kDelta);
        assertEquals(0, tagInCameraCoords.getRotation().getY(), kDelta);
        assertEquals(0, tagInCameraCoords.getRotation().getZ(), kDelta);

        // "raw" layout, which is "out of the page" tag orientation.j
        // which is WRONG WRONG WRONG
        Path path = Filesystem.getDeployDirectory().toPath().resolve("2024-crescendo.json");
        AprilTagFieldLayout layout = new AprilTagFieldLayout(path);
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        Pose3d tagInFieldCoords = layout.getTagPose(7).get();

        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(tagInCameraCoords, tagInFieldCoords);

        // notice this is WRONG WRONG WRONG because the raw tag rotation is also WRONG
        assertEquals(-1.038, cameraInFieldCoords.getX(), kDelta);
        // the tag is over to the left
        assertEquals(5.548, cameraInFieldCoords.getY(), kDelta);
        // tag center is about 57 inches up
        assertEquals(1.451, cameraInFieldCoords.getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getY(), kDelta);
        // camera is facing down field which is WRONG WRONG WRONG
        assertEquals(0, cameraInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    void testTagRotationCorrect24() throws IOException {
        // this illustrates the CORRECT tag orientation.

        // say we're playing blue, on the blue side, looking at tag 7.
        // the robot is facing 180, and the camera returns this.
        // note that the camera code returns the identity rotation when
        // it's looking straight at a tag, which implies "into the page"
        // orientation.

        Blip24 blip = new Blip24(7,
                new Transform3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, 0)));

        Transform3d tagInCameraCoords = PoseEstimationHelper.blipToTransform(blip);
        assertEquals(1, tagInCameraCoords.getX(), kDelta);
        assertEquals(0, tagInCameraCoords.getY(), kDelta);
        assertEquals(0, tagInCameraCoords.getZ(), kDelta);
        assertEquals(0, tagInCameraCoords.getRotation().getX(), kDelta);
        assertEquals(0, tagInCameraCoords.getRotation().getY(), kDelta);
        assertEquals(0, tagInCameraCoords.getRotation().getZ(), kDelta);

        // first try the "corrected" layout, which is "into the page" tag orientation.
        // this is CORRECT
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        Pose3d tagInFieldCoords = layout.getTagPose(Alliance.Blue, 7).get();

        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(tagInCameraCoords, tagInFieldCoords);

        // the tag is a little bit behind the line, so we're a little closer to the line
        // than 1m.
        assertEquals(0.9619, cameraInFieldCoords.getX(), kDelta);
        // the tag is over to the left; so is the camera
        assertEquals(5.548, cameraInFieldCoords.getY(), kDelta);
        // tag center is about 57 inches up; so is the camera
        assertEquals(1.451, cameraInFieldCoords.getZ(), kDelta);
        // zero roll
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        // zero tilt
        assertEquals(0, cameraInFieldCoords.getRotation().getY(), kDelta);
        // camera is facing back towards the wall
        assertEquals(Math.PI, cameraInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    void testAngleToTarget() {
        {
            Transform3d cameraInRobotCoordinates = new Transform3d();
            Blip24 blip = new Blip24(7, new Transform3d(0, 0, 1, new Rotation3d()));
            Transform3d t = PoseEstimationHelper.toTarget(cameraInRobotCoordinates, blip);
            assertEquals(0, t.getTranslation().toTranslation2d().getAngle().getRadians(), kDelta);
            assertEquals(1, t.getTranslation().toTranslation2d().getNorm(), kDelta);
        }
        {
            Transform3d cameraInRobotCoordinates = new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI / 2));
            Blip24 blip = new Blip24(7, new Transform3d(0, 0, 1, new Rotation3d()));
            Transform3d t = PoseEstimationHelper.toTarget(cameraInRobotCoordinates, blip);
            assertEquals(Math.PI / 2, t.getTranslation().toTranslation2d().getAngle().getRadians(), kDelta);
            assertEquals(1, t.getTranslation().toTranslation2d().getNorm(), kDelta);
        }
        {
            Transform3d cameraInRobotCoordinates = new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI / 2));
            Blip24 blip = new Blip24(7, new Transform3d(1, 0, 1, new Rotation3d()));
            Transform3d t = PoseEstimationHelper.toTarget(cameraInRobotCoordinates, blip);
            assertEquals(Math.PI / 4, t.getTranslation().toTranslation2d().getAngle().getRadians(), kDelta);
            assertEquals(1.414, t.getTranslation().toTranslation2d().getNorm(), kDelta);
        }
        {
            // this very strange camera angle is 90 tilt up, then 90 around (global) z, so
            // the right side of the camera is pointing forward
            Transform3d cameraInRobotCoordinates = new Transform3d(0, 0, 0,
                    new Rotation3d(0, -Math.PI / 2, Math.PI / 2));
            // this should end up straight ahead
            Blip24 blip = new Blip24(7, new Transform3d(1, 0, 1, new Rotation3d()));
            Transform3d t = PoseEstimationHelper.toTarget(cameraInRobotCoordinates, blip);
            assertEquals(0, t.getTranslation().toTranslation2d().getAngle().getRadians(), kDelta);
            assertEquals(1, t.getTranslation().toTranslation2d().getNorm(), kDelta);
        }
    }

}
