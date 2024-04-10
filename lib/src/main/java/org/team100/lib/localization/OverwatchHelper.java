package org.team100.lib.localization;

import org.team100.lib.config.Camera;
import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

/** Static methods used for the Overwatch feature. */
public class OverwatchHelper {

    public static Transform3d tagInFieldCoordinates(
            String cameraSerialNumber,
            Blip24 blip) {
        // in the overwatch case the camera offset is measured from the field origin
        final Transform3d cameraInFieldCoordinates = Camera.get(cameraSerialNumber).getOffset();
        final Transform3d tagInCameraCoords = PoseEstimationHelper.blipToTransform(blip);
        return tagInFieldCoordinates(cameraInFieldCoordinates, tagInCameraCoords);
    }

    /**
     * In the overwatch case the camera offset is measured from the field origin
     * 
     * @param cameraInFieldCoordinates
     * @param blip
     */
    public static Transform3d tagInFieldCoordinates(
            Transform3d cameraInFieldCoordinates,
            Blip24 blip) {
        final Transform3d tagInCameraCoords = PoseEstimationHelper.blipToTransform(blip);
        return cameraInFieldCoordinates.plus(tagInCameraCoords);
    }

    public static Transform3d tagInFieldCoordinates(
            Transform3d cameraInFieldCoordinates,
            Transform3d tagInCameraCoords) {
        return cameraInFieldCoordinates.plus(tagInCameraCoords);
    }

    public static Pose2d toPose(Transform3d transform) {
        return GeometryUtil.kPose3dZero.plus(transform).toPose2d();
    }

    private OverwatchHelper() {
        //
    }
}
