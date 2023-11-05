package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Figure out how to correct for pan and tilt
 *
 * Same diagram as VisionTransformTest.
 *
 * Test case where the robot and the tag are at opposite corners of a square.
 * The robot is at the origin, which is at the lower right corner, facing the
 * opposite corner. The tag is at the upper left, visible from below. Tag
 * orientation "into the page" is up.
 * 
 * .*
 * .* facing up
 * .*
 * TAG................. (1, 0)
 * ....................
 * ....................
 * ....................
 * ....................
 * .............*......
 * ...............*.... facing the tag
 * .................*..
 * .................ROBOT (0,0)
 * (1,0)
 * 
 * tag pose is R(0)|t(1,1)
 * robot pose is R(pi/4)|t(0,0)
 */
class PanTiltOffsetTest {
    private static final double kDelta = 0.01;

    public PanTiltOffsetTest() throws IOException {
        // load the JNI
        CameraServerCvJNI.forceLoad();
    }

    /**
     * Correct for offset but the offset is zero.
     */
    @Test
    void testZeroOffset() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(),
                new Rotation3d());

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);

        // camera input; we ignore the rotation from the camera.
        Translation3d tagTranslationInCameraCoords = new Translation3d(Math.sqrt(2), 0, 0);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // check that the derived tag rotation is correct
        assertEquals(-Math.PI / 4, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(0, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * Dolly back 0.5m behind the robot center.
     */
    @Test
    void testDolly() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(-0.5, 0, 0),
                new Rotation3d());

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);

        // camera input; we ignore the rotation from the camera.
        // because the camera happens to be pointing at the tag, the distance is simply
        // longer by the amount of the offset
        Translation3d tagTranslationInCameraCoords = new Translation3d(Math.sqrt(2) + 0.5, 0, 0);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // check that the derived tag rotation is correct
        assertEquals(-Math.PI / 4, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(-Math.sqrt(2) / 4, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(-Math.sqrt(2) / 4, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * Pan 45 degrees to the left.
     */
    @Test
    void testPan() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, 0, Math.PI / 4));

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);

        // camera input; we ignore the rotation from the camera.
        // because of the pan, the translation is different.
        Translation3d tagTranslationInCameraCoords = new Translation3d(1, -1, 0);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // check that the derived tag rotation is correct
        // because of the pan, the rotation is different.
        assertEquals(-Math.PI / 2, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        // should be at the origin but looking 90 degrees left
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(0, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 2, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * Truck sqrt(2)m left.
     */
    @Test
    void testTruck() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(0, Math.sqrt(2), 0),
                new Rotation3d(0, 0, 0));

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);

        // camera input; we ignore the rotation from the camera.
        // because of the truck, the translation is different.
        Translation3d tagTranslationInCameraCoords = new Translation3d(Math.sqrt(2), -Math.sqrt(2), 0);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // check that the derived tag rotation is correct
        // because of the pan, the rotation is different.
        assertEquals(-Math.PI / 4, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        // should be moved to the left
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(-1, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(1, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * Moved and rotated, all in 2d.
     */
    @Test
    void testAll2d() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(Math.sqrt(2), Math.sqrt(2), 0),
                new Rotation3d(0, 0, -Math.PI / 2));

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);

        // camera input; we ignore the rotation from the camera.
        // because of the truck, the translation is different.
        Translation3d tagTranslationInCameraCoords = new Translation3d(Math.sqrt(2), 0, 0);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // check that the derived tag rotation is correct
        // because of the pan, the rotation is different.
        assertEquals(Math.PI / 4, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        // should be moved to the left
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(0, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(2, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(-Math.PI / 4, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * Tilt 45 degrees up.
     */
    @Test
    void testTilt() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, -Math.PI / 4, 0)); // upward tilt is negative pitch

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);

        // do we still get the correct pan and tilt values from the combined rotation?
        // the "normal" order is Tate-Bryant so yaw then pitch then roll.
        assertEquals(0, cameraRotationInFieldCoords.getX(), kDelta);
        assertEquals(-Math.PI / 4, cameraRotationInFieldCoords.getY(), kDelta);
        assertEquals(Math.PI / 4, cameraRotationInFieldCoords.getZ(), kDelta);

        // camera input; we ignore the rotation from the camera.
        // because of the tilt, the range (x) is more, and there's downward offset
        Translation3d tagTranslationInCameraCoords = new Translation3d(1, 0, -1);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // do we get the right tag rotation?
        // negative roll in camera frame, not sure this is right.
        assertEquals(-0.615, tagRotationInCameraCoords.getX(), kDelta);
        // positive pitch in camera frame, not sure this is right.
        assertEquals(0.524, tagRotationInCameraCoords.getY(), kDelta);
        // negative yaw in camera frame, not sure this is right.
        assertEquals(-0.955, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        // should be at the origin, panned like the robot, and tilted 45 deg up
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(0, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        // upward tilt
        assertEquals(-Math.PI / 4, cameraInFieldCoords.getRotation().getY(), kDelta);
        // same yaw as robot
        assertEquals(Math.PI / 4, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * Offset to the left, pan to face the tag, and tilt 45 degrees up. tag view
     * should be a simple tilt from there.
     */
    @Test
    void testTiltWithPandAndOffset() {
        // in robot coords the camera rotation is both tilt and pan
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(0, Math.sqrt(2), 0),
                new Rotation3d(0, -Math.PI / 4, -Math.PI / 4));

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);

        // do we still get the correct pan and tilt values from the combined rotation?
        // the "normal" order is Tate-Bryant so yaw then pitch then roll.
        // in field coords the camera rotation is pure tilt
        assertEquals(0, cameraRotationInFieldCoords.getX(), kDelta);
        assertEquals(-Math.PI / 4, cameraRotationInFieldCoords.getY(), kDelta);
        assertEquals(0, cameraRotationInFieldCoords.getZ(), kDelta);

        // camera input; we ignore the rotation from the camera.
        // because of the tilt, the range (x) is more, and there's downward offset
        Translation3d tagTranslationInCameraCoords = new Translation3d(Math.sqrt(2), 0, -Math.sqrt(2));

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // do we get the right tag rotation?
        // should be positive pitch
        assertEquals(0, tagRotationInCameraCoords.getX(), kDelta); //
        assertEquals(Math.PI / 4, tagRotationInCameraCoords.getY(), kDelta);
        assertEquals(0, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(-1, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(1, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(-Math.PI / 4, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * semi-realistic example
     * tag is at R(0,0,PI)t(1,4,1)
     * robot is at R(0,0,-3PI/4)t(3,3,0)
     * camera offset is R(0,-PI/4,-PI/4)t(0,-sqrt(2),1)
     * so tag in camera view should be R(0, PI/4, 0)t(sqrt(2)/2,0-sqrt(2)/2)
     */
    @Test
    void testSemiRealisticExample() {
        // in robot coords the camera rotation is both tilt and pan
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(0, -Math.sqrt(2), 1),
                new Rotation3d(0, -Math.PI / 4, -Math.PI / 4));

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, -3.0 * Math.PI / 4.0);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);

        // do we still get the correct pan and tilt values from the combined rotation?
        // the "normal" order is Tate-Bryant so yaw then pitch then roll.
        // in field coords the camera rotation is pure tilt
        assertEquals(0, cameraRotationInFieldCoords.getX(), kDelta);
        assertEquals(-Math.PI / 4, cameraRotationInFieldCoords.getY(), kDelta);
        assertEquals(-Math.PI, cameraRotationInFieldCoords.getZ(), kDelta);

        // camera input; we ignore the rotation from the camera.
        // because of the tilt, the range (x) is more, and there's downward offset
        Translation3d tagTranslationInCameraCoords = new Translation3d(Math.sqrt(2) / 2, 0, -Math.sqrt(2) / 2);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 4, 1, new Rotation3d(0, 0, Math.PI));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // do we get the right tag rotation?
        // should be positive pitch
        assertEquals(0, tagRotationInCameraCoords.getX(), kDelta); //
        assertEquals(Math.PI / 4, tagRotationInCameraCoords.getY(), kDelta);
        assertEquals(0, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(2, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(4, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(1, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(-Math.PI / 4, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(-Math.PI, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(3, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(3, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(-3.0 * Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * semi-realistic example with blip input
     * tag is at R(0,0,PI)t(1,4,1)
     * robot is at R(0,0,-3PI/4)t(3,3,0)
     * camera offset is R(0,-PI/4,-PI/4)t(0,-sqrt(2),1)
     * so tag in camera view should be R(0, PI/4, 0)t(sqrt(2)/2,0-sqrt(2)/2)
     */
    @Test
    void testSemiRealisticExampleWithBlips() {
        double rot = Math.sqrt(2) / 2;
        Blip blip = new Blip(5,
                new double[][] { // pure tilt
                        { 1, 0, 0 },
                        { 0, rot, -rot },
                        { 0, rot, rot } },
                new double[][] { // one meter range (Z forward)
                        { 0 },
                        { Math.sqrt(2) / 2 },
                        { Math.sqrt(2) / 2 } });

        // Translation3d blipTranslation = VisionDataProvider.blipToTranslation(blip);
        Translation3d tagTranslationInCameraCoords = PoseEstimationHelper.blipToTranslation(blip);

        assertEquals(Math.sqrt(2) / 2, tagTranslationInCameraCoords.getX(), kDelta);
        assertEquals(0, tagTranslationInCameraCoords.getY(), kDelta);
        assertEquals(-Math.sqrt(2) / 2, tagTranslationInCameraCoords.getZ(), kDelta);

        // in robot coords the camera rotation is both tilt and pan
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(0, -Math.sqrt(2), 1),
                new Rotation3d(0, -Math.PI / 4, -Math.PI / 4));

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, -3.0 * Math.PI / 4.0);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);

        // do we still get the correct pan and tilt values from the combined rotation?
        // the "normal" order is Tate-Bryant so yaw then pitch then roll.
        // in field coords the camera rotation is pure tilt
        assertEquals(0, cameraRotationInFieldCoords.getX(), kDelta);
        assertEquals(-Math.PI / 4, cameraRotationInFieldCoords.getY(), kDelta);
        assertEquals(-Math.PI, cameraRotationInFieldCoords.getZ(), kDelta);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 4, 1, new Rotation3d(0, 0, Math.PI));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // do we get the right tag rotation?
        // should be positive pitch
        assertEquals(0, tagRotationInCameraCoords.getX(), kDelta); //
        assertEquals(Math.PI / 4, tagRotationInCameraCoords.getY(), kDelta);
        assertEquals(0, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(2, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(4, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(1, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(-Math.PI / 4, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(-Math.PI, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(3, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(3, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(-3.0 * Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * Blip from the network tables listener.
     * This represents a tag with 45 deg tilted rotation and diagonal translation.
     */
    private Blip getBlip() {
        double rot = Math.sqrt(2) / 2;
        Blip blip = new Blip(5,
                new double[][] { // pure tilt
                        { 1, 0, 0 },
                        { 0, rot, -rot },
                        { 0, rot, rot } },
                new double[][] { // one meter range (Z forward)
                        { 0 },
                        { Math.sqrt(2) / 2 },
                        { Math.sqrt(2) / 2 } });
        return blip;
    }

    // from the gyro
    private Rotation3d getYawMeasurement() {
        return new Rotation3d(0, 0, -3.0 * Math.PI / 4.0);
    }

    // configuration: camera offset
    private Transform3d getCameraOffset(int cameraIdentity /* ignored */) {
        return new Transform3d(
                new Translation3d(0, -Math.sqrt(2), 1),
                new Rotation3d(0, -Math.PI / 4, -Math.PI / 4));
    }

    /** Returns a tag at (1,4,1) and rotated 180, i.e. in our own scoring area. */
    private Pose3d tagInFieldCoords(int tagid /* ignored */) {
        return new Pose3d(1, 4, 1, new Rotation3d(0, 0, Math.PI));
    }

    /**
     * semi-realistic example with blip input
     * tag is at R(0,0,PI)t(1,4,1)
     * robot is at R(0,0,-3PI/4)t(3,3,0)
     * camera offset is R(0,-PI/4,-PI/4)t(0,-sqrt(2),1)
     * so tag in camera view should be R(0, PI/4, 0)t(sqrt(2)/2,0-sqrt(2)/2)
     */
    @Test
    void testSemiRealisticExampleWithBlipsUsingSingleFunction() {
        // CONFIGURATION

        // in robot coords the camera rotation is both tilt and pan
        final Transform3d cameraInRobotCoords = getCameraOffset(0); // "camera zero"

        // tag pose
        final Pose3d tagInFieldCoords = tagInFieldCoords(0); // "tag zero"

        // INPUTS

        // from the network tables listener
        Blip blip = getBlip();

        // from the gyro
        Rotation3d robotRotationInFieldCoordsFromGyro = getYawMeasurement();

        // CALCULATIONS

        Pose3d robotInFieldCoords = PoseEstimationHelper.getRobotPoseInFieldCoords(
                cameraInRobotCoords,
                tagInFieldCoords,
                blip,
                robotRotationInFieldCoordsFromGyro);

        assertEquals(3, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(3, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(-3.0 * Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    void testSemiRealisticExampleWithBlipsUsingSingleFunctionUsingCameraRotation() {
        {
            // see diagram at the top of the file.
            // tag pose is R(0)|t(1,1)
            // robot pose is R(pi/4)|t(0,0)
            // start with no offset
            final Transform3d cameraInRobotCoords = new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d());

            // tag facing 0, at 1,1,0
            final Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

            double rot = Math.sqrt(2) / 2;
            Blip blip = new Blip(5,
                    new double[][] { // pan right in camera frame = +y rot
                            { rot, 0, rot },
                            { 0, 1, 0 },
                            { -rot, 0, rot } },
                    new double[][] { // unit-square-diagonal range
                            { 0 },
                            { 0 },
                            { Math.sqrt(2) } });

            Pose3d robotInFieldCoords = PoseEstimationHelper.getRobotPoseInFieldCoords(
                    cameraInRobotCoords,
                    tagInFieldCoords,
                    blip);

            assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
            assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
            assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
            assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
            assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
            assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
        }
        {
            // camera tilt
            final Transform3d cameraInRobotCoords = new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(0, -Math.PI / 4, 0)); // tilt 45 up

            // tag facing 0, at 1,0,1
            final Pose3d tagInFieldCoords = new Pose3d(1, 0, 1, new Rotation3d(0, 0, 0));

            double rot = Math.sqrt(2) / 2;
            Blip blip = new Blip(5,
                    new double[][] { // tag tilts away, we're looking up at it in camera frame = -x rot
                            { 1, 0, 0 },
                            { 0, rot, rot },
                            { 0, -rot, rot } },
                    new double[][] { // unit-square-diagonal range
                            { 0 },
                            { 0 },
                            { Math.sqrt(2) } });

            Pose3d robotInFieldCoords = PoseEstimationHelper.getRobotPoseInFieldCoords(
                    cameraInRobotCoords,
                    tagInFieldCoords,
                    blip);

            assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
            assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
            assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
            assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
            assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
            assertEquals(0, robotInFieldCoords.getRotation().getZ(), kDelta);
        }
    }
}
