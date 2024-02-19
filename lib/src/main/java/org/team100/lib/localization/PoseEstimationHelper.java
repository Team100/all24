package org.team100.lib.localization;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

/**
 * Static methods used to interpret camera input.
 */
public class PoseEstimationHelper {
    private static final Telemetry t = Telemetry.get();
    private static final String m_name = Names.name(PoseEstimationHelper.class);

    /**
     * Calculate robot pose.
     * 
     * First calculates the distance to the tag. If it's closer than the threshold,
     * use the camera-derived tag rotation. If it's far, use the gyro.
     */
    public static Pose3d getRobotPoseInFieldCoords(
            Transform3d cameraInRobotCoords,
            Pose3d tagInFieldCoords,
            Blip blip,
            Rotation3d robotRotationInFieldCoordsFromGyro,
            double thresholdMeters) {

        Translation3d tagTranslationInCameraCoords = blipToTranslation(blip);

        if (tagTranslationInCameraCoords.getNorm() < thresholdMeters) {
            t.log(Level.DEBUG, m_name, "rotation_source", "CAMERA");
            return getRobotPoseInFieldCoords(
                    cameraInRobotCoords,
                    tagInFieldCoords,
                    blip);
        }

        t.log(Level.DEBUG, m_name, "rotation_source", "GYRO");

        return getRobotPoseInFieldCoords(
                cameraInRobotCoords,
                tagInFieldCoords,
                blip,
                robotRotationInFieldCoordsFromGyro);
    }

    /**
     * ALERT: the new python code uses a different result type so this might be
     * wrong.
     * 
     * TODO: check that the resulting transform is correct.
     */
    public static Pose3d getRobotPoseInFieldCoords(
            Transform3d cameraInRobotCoords,
            Pose3d tagInFieldCoords,
            Blip24 blip,
            Rotation3d robotRotationInFieldCoordsFromGyro,
            double thresholdMeters) {

        Translation3d tagTranslationInCameraCoords = blipToTranslation(blip);

        if (tagTranslationInCameraCoords.getNorm() < thresholdMeters) {
            t.log(Level.DEBUG, m_name, "rotation_source", "CAMERA");
            return getRobotPoseInFieldCoords(
                    cameraInRobotCoords,
                    tagInFieldCoords,
                    blip);
        }

        t.log(Level.DEBUG, m_name, "rotation_source", "GYRO");

        return getRobotPoseInFieldCoords(
                cameraInRobotCoords,
                tagInFieldCoords,
                blip,
                robotRotationInFieldCoordsFromGyro);
    }

    /**
     * Calculate robot pose.
     * 
     * Given the blip and its corresponding field location, and the camera offset,
     * return the robot pose in field coordinates.
     * 
     * This method trusts the tag rotation calculated by the camera.
     */
    public static Pose3d getRobotPoseInFieldCoords(
            Transform3d cameraInRobotCoords,
            Pose3d tagInFieldCoords,
            Blip blip) {
        Transform3d tagInCameraCoords = blipToTransform(blip);
        Pose3d cameraInFieldCoords = toFieldCoordinates(tagInCameraCoords, tagInFieldCoords);
        return applyCameraOffset(cameraInFieldCoords, cameraInRobotCoords);
    }

    /**
     * ALERT: the new python code uses a different result type so this might be
     * wrong.
     * 
     * TODO: check that the resulting transform is correct.
     */
    public static Pose3d getRobotPoseInFieldCoords(
            Transform3d cameraInRobotCoords,
            Pose3d tagInFieldCoords,
            Blip24 blip) {
                
        Transform3d tagInCameraCoords = blipToTransform(blip);
        Pose3d cameraInFieldCoords = toFieldCoordinates(tagInCameraCoords, tagInFieldCoords);
        return applyCameraOffset(cameraInFieldCoords, cameraInRobotCoords);
    }

    /**
     * Calculate robot pose.
     * 
     * Given the blip, the heading, the camera offset, and the absolute tag pose,
     * return the absolute robot pose in field coordinates.
     * 
     * This method does not trust the tag rotation from the camera, it uses the gyro
     * signal instead.
     * 
     * @param cameraInRobotCoords                camera offset expressed as a
     *                                           transform from robot-frame to
     *                                           camera-frame, e.g.camera 0.5m in
     *                                           front of the robot center and 0.5m
     *                                           from the floor would have a
     *                                           translation (0.5, 0, 0.5)
     * @param tagInFieldCoords                   tag location expressed as a pose in
     *                                           field-frame. this should come from
     *                                           the json
     * @param blip                               direct from the camera
     * @param robotRotationInFieldCoordsFromGyro direct from the gyro. note that
     *                                           drive.getPose() isn't exactly the
     *                                           gyro reading; it might be better to
     *                                           use the real gyro than the getPose
     *                                           method.
     */
    public static Pose3d getRobotPoseInFieldCoords(
            Transform3d cameraInRobotCoords,
            Pose3d tagInFieldCoords,
            Blip blip,
            Rotation3d robotRotationInFieldCoordsFromGyro) {
        Rotation3d cameraRotationInFieldCoords = cameraRotationInFieldCoords(
                cameraInRobotCoords,
                robotRotationInFieldCoordsFromGyro);
        Translation3d tagTranslationInCameraCoords = blipToTranslation(blip);
        Rotation3d tagRotationInCameraCoords = tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);
        Pose3d cameraInFieldCoords = toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        return applyCameraOffset(
                cameraInFieldCoords,
                cameraInRobotCoords);
    }

    /**
     * ALERT: the new python code uses a different result type so this might be
     * wrong.
     * 
     * TODO: check that the resulting pose is correct.
     */
    public static Pose3d getRobotPoseInFieldCoords(
            Transform3d cameraInRobotCoords,
            Pose3d tagInFieldCoords,
            Blip24 blip,
            Rotation3d robotRotationInFieldCoordsFromGyro) {

        Rotation3d cameraRotationInFieldCoords = cameraRotationInFieldCoords(
                cameraInRobotCoords,
                robotRotationInFieldCoordsFromGyro);
                
        Translation3d tagTranslationInCameraCoords = blipToTranslation(blip);
        
        t.log(Level.DEBUG, m_name, "CAMERA ROT IN FIELD COORDS", cameraRotationInFieldCoords.toRotation2d());
        t.log(Level.DEBUG, m_name, "TAG TRANSLATION IN CAM COORDS", tagTranslationInCameraCoords.toTranslation2d());
        
        // System.out.println("TAG TRANLSAION IN CAM COORDS :" +  tagTranslationInCameraCoords.toTranslation2d());
        // System.out.println("CAMERA ROT IN FIELD COORDS: " + cameraRotationInFieldCoords.toRotation2d());  
              
        // System.out.println("TAG IN FIELD COORDS COOORDS"+ tagInFieldCoords.toPose2d());        

        Rotation3d tagRotationInCameraCoords = tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        t.log(Level.DEBUG, m_name, "TAG ROTATION IN CAM COOORDS", tagRotationInCameraCoords.toRotation2d());
    

        // System.out.println("TAG ROTATION IN CAM COOORDS"+ tagRotationInCameraCoords.toRotation2d());        

        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);
    

        Pose3d cameraInFieldCoords = toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        
        // System.out.println("CAM IN FIELD COORDS:::: " + cameraInFieldCoords.toPose2d());
        t.log(Level.DEBUG, m_name, "CAM IN FIELD COORDS", cameraInFieldCoords.getTranslation().toTranslation2d());

        return applyCameraOffset(
                cameraInFieldCoords,
                cameraInRobotCoords);
    }

    //////////////////////////////
    //
    // package private below, don't use these.

    /**
     * given the gyro rotation and the camera offset, return the camera absolute
     * rotation. Package-private for testing.
     */
    static Rotation3d cameraRotationInFieldCoords(
            Transform3d cameraInRobotCoords,
            Rotation3d robotRotationInFieldCoordsFromGyro) {
        return cameraInRobotCoords.getRotation()
                .rotateBy(robotRotationInFieldCoordsFromGyro);
    }

    /**
     * Extract translation and rotation from z-forward blip and return the same
     * translation and rotation as an NWU x-forward transform. Package-private for
     * testing.
     */
    static Transform3d blipToTransform(Blip b) {
        return new Transform3d(blipToTranslation(b), blipToRotation(b));
    }

    /**
     * ALERT: the new python code uses a different result type so this might be
     * wrong.
     * 
     * TODO: check that the resulting transform is correct.
     * 
     * @param b
     * @return
     */
    static Transform3d blipToTransform(Blip24 b) {
        return new Transform3d(blipToTranslation(b), blipToRotation(b));
    }

    /**
     * Extract the translation from a "z-forward" blip and return the same
     * translation expressed in our usual "x-forward" NWU translation.
     * It would be possible to also consume the blip rotation matrix, if it were
     * renormalized, but it's not very accurate, so we don't consume it.
     * Package-private for testing.
     */
    static Translation3d blipToTranslation(Blip b) {
        return new Translation3d(b.pose_t[2][0], -1.0 * b.pose_t[0][0], -1.0 * b.pose_t[1][0]);
    }

    /**
     * ALERT: the new python code produces a different result type so this might be
     * wrong.
     * 
     * TODO: check that the resulting translation orientation is correct
     * 
     * @param b
     * @return
     */
    static Translation3d blipToTranslation(Blip24 b) {
        return GeometryUtil.zForwardToXForward(b.getPose().getTranslation());
    }

    /**
     * Extract the rotation from the "z forward" blip and return the same rotation
     * expressed in our usual "x forward" NWU coordinates. Package-private for
     * testing.
     */
    static Rotation3d blipToRotation(Blip b) {
        Mat rmat = new Mat(3, 3, CvType.CV_64F);
        rmat.put(0, 0, b.pose_R[2][2]);
        rmat.put(0, 1, -b.pose_R[2][0]);
        rmat.put(0, 2, -b.pose_R[2][1]);

        rmat.put(1, 0, -b.pose_R[0][2]);
        rmat.put(1, 1, b.pose_R[0][0]);
        rmat.put(1, 2, b.pose_R[0][1]);

        rmat.put(2, 0, -b.pose_R[1][2]);
        rmat.put(2, 1, b.pose_R[1][0]);
        rmat.put(2, 2, b.pose_R[1][1]);

        // convert it to axis-angle
        Mat rvec = new Mat(3, 1, CvType.CV_64F);
        Calib3d.Rodrigues(rmat, rvec);

        // convert back to rotation matrix -- this should be orthogonal
        Mat rmat2 = new Mat();
        Calib3d.Rodrigues(rvec, rmat2);

        Matrix<N3, N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                matrix.set(row, col, rmat2.get(row, col)[0]);
            }
        }

        return new Rotation3d(matrix);
    }

    /**
     * ALERT! the new python code uses a different estimator which may (probably)
     * use a different convention for rotation -- the original AprilTag library uses
     * Z-into-tag whereas I think the WPI convention is Z-out-of-tag.
     * 
     * it doesn't matter that much because we don't actually use the tag rotation
     * for anything (we use the gyro instead), but it would be good for it to be
     * correct.
     * 
     * TODO: verify the polarity of this rotation
     * 
     * @param b
     * @return
     */
    static Rotation3d blipToRotation(Blip24 b) {
        return GeometryUtil.zForwardToXForward(b.getPose().getRotation());
    }

    /**
     * Because the camera's estimate of tag rotation isn't very accurate, this
     * synthesizes an estimate using the tag rotation in field frame (from json) and
     * the camera rotation in field frame (from gyro). Package-private for testing.
     */
    static Rotation3d tagRotationInRobotCoordsFromGyro(
            Rotation3d tagRotationInFieldCoords,
            Rotation3d cameraRotationInFieldCoords) {
        return tagRotationInFieldCoords.rotateBy(cameraRotationInFieldCoords.unaryMinus());
    }

    /**
     * Given the tag in camera frame and tag in field frame, return the camera in
     * field frame. Package-private for testing.
     */
    static Pose3d toFieldCoordinates(Transform3d tagInCameraCords, Pose3d tagInFieldCords) {
        // First invert the camera-to-tag transform, obtaining tag-to-camera.
        Transform3d cameraInTagCords = tagInCameraCords.inverse();
        // Transform3d cameraInTagCords = tagInCameraCords;

        // Then compose field-to-tag with tag-to-camera to get field-to-camera.
        return tagInFieldCords.transformBy(cameraInTagCords);
    }

    /**
     * Given the camera in field frame and camera in robot frame, return the robot
     * in field frame. Package-private for testing.
     */
    static Pose3d applyCameraOffset(Pose3d cameraInFieldCoords, Transform3d cameraInRobotCoords) {
        Transform3d robotInCameraCoords = cameraInRobotCoords.inverse();
        return cameraInFieldCoords.transformBy(robotInCameraCoords);
    }

    private PoseEstimationHelper() {
    }
}
