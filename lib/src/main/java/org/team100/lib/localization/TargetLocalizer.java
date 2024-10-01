package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class TargetLocalizer {

    /**
     * Converts camera-relative sights to field relative translations.
     * 
     * NOTE! camera sights are x-ahead WPI coordinates, not z-ahead camera
     * coordinates.
     */
    public static List<Translation2d> cameraRotsToFieldRelativeArray(
            Pose2d robotPose,
            Transform3d cameraInRobotCoordinates,
            Rotation3d[] sights) {
        ArrayList<Translation2d> Tnotes = new ArrayList<>();
        for (Rotation3d note : sights) {
            cameraRotToFieldRelative(
                    robotPose,
                    cameraInRobotCoordinates,
                    note).ifPresent(Tnotes::add);
        }
        return Tnotes;
    }

    public static Optional<Translation2d> cameraRotToFieldRelative(
            Pose2d robotPose,
            Transform3d cameraInRobotCoordinates,
            Rotation3d note) {
        //
        // this appears to have filtered out very close poses, which i think is the
        // opposite of what it is supposed to do: avoid the horizon.
        // i'm kinda suspicious how this ever worked. there was no unit test
        // for it as of Jul 2024.
        // if (note.getY() < cameraInRobotCoordinates.getRotation().getY()) {
        //
        Optional<Translation2d> robotRelative = TargetLocalizer
                .sightToRobotRelative(cameraInRobotCoordinates, note);
        if (robotRelative.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(PoseEstimationHelper.robotRelativeToFieldRelative(
                robotPose,
                robotRelative.get()));
    }

    /**
     * Return the robot-relative intersection of the camera-relative target sight
     * with the floor.
     */
    public static Optional<Translation2d> sightToRobotRelative(
            Transform3d cameraInRobotCoordinates,
            Rotation3d sight) {
        return TargetLocalizer.sightInRobotCoordsToTranslation2d(
                TargetLocalizer.sightInRobotCoords(cameraInRobotCoordinates, sight));
    }

    /**
     * Convert a camera-relative sight (with no translational component) into a
     * robot-relative sight.
     */
    public static Transform3d sightInRobotCoords(
            Transform3d cameraInRobotCoordinates,
            Rotation3d sight) {
        Transform3d sightTransform = new Transform3d(0, 0, 0, sight);
        return cameraInRobotCoordinates.plus(sightTransform);
    }

    /**
     * Return the robot-relative intersection of the robot-relative target sight
     * with the floor.
     */
    public static Optional<Translation2d> sightInRobotCoordsToTranslation2d(
            Transform3d robotRelativeSight) {
        double h = robotRelativeSight.getZ();
        if (h <= 0) {
            // camera is below the floor
            return Optional.empty();
        }
        double yaw = robotRelativeSight.getRotation().getZ();
        double pitch = robotRelativeSight.getRotation().getY();
        if (pitch <= 0) {
            // above the horizon
            return Optional.empty();
        }
        double d = h / Math.tan(pitch);
        double x = robotRelativeSight.getX();
        double y = robotRelativeSight.getY();
        return Optional.of(new Translation2d(
                x + d * Math.cos(yaw),
                y + d * Math.sin(yaw)));
    }

    private TargetLocalizer() {
        //
    }
}
