package org.team100.frc2024.motion;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AmpUtil {

    private static boolean isInsideRectangle(
            Translation2d translation2d,
            Translation2d rectangle,
            double width,
            double height) {
        return (translation2d.getX() >= rectangle.getX() && translation2d.getX() <= rectangle.getX() + height)
                && (translation2d.getY() >= rectangle.getY() && translation2d.getY() <= rectangle.getY() + width);
    }

    public static List<Pose2d> getShortestTrajecNew(SwerveDriveSubsystem drive) {
        Map<String, List<Pose2d>> poseMap = new ConcurrentHashMap<>();
        Translation2d currentTranslation = drive.getPose().getTranslation();
        Pose2d currentPose = drive.getPose();

        List<Pose2d> rightSideFarStage = new ArrayList<>();

        List<Pose2d> leftSideFarStage = new ArrayList<>();

        List<Pose2d> centerFarStage = new ArrayList<>();

        List<Pose2d> sideCloseStage = new ArrayList<>();

        List<Pose2d> ampDirect = new ArrayList<>();

        if (isInsideRectangle(currentTranslation, new Translation2d(10, 0), 8, 6)) { // Inside Far Wing
            rightSideFarStage.add(new Pose2d(10.009935, 2.312511, new Rotation2d()));
            rightSideFarStage.add(new Pose2d(7.682685, 6.063078, new Rotation2d()));
            rightSideFarStage.add(new Pose2d(1.715115, 7.334519, new Rotation2d()));

            centerFarStage.add(new Pose2d(12.342509, 3.421028, new Rotation2d()));
            centerFarStage.add(new Pose2d(11.693578, 4.055302, new Rotation2d()));
            centerFarStage.add(new Pose2d(10.903611, 4.329293, new Rotation2d()));
            centerFarStage.add(new Pose2d(7.682685, 6.063078, new Rotation2d()));
            centerFarStage.add(new Pose2d(1.715115, 7.334519, new Rotation2d()));

            leftSideFarStage.add(new Pose2d(11.255004, 6.867042, new Rotation2d()));
            leftSideFarStage.add(new Pose2d(1.715115, 7.334519, new Rotation2d()));

        } else if (isInsideRectangle(currentTranslation, new Translation2d(0, 0), 8, 6)) { // Inside Close Wing
            ampDirect.add(new Pose2d(1.715115, 7.334519, new Rotation2d()));
        } else {
            sideCloseStage.add(new Pose2d(5.885868, 6.359865, new Rotation2d()));
            sideCloseStage.add(new Pose2d(1.715115, 7.334519, new Rotation2d()));
        }

        poseMap.put("Right Side Far Stage", rightSideFarStage);
        poseMap.put("Center Far Stage", centerFarStage);
        poseMap.put("Side Close Stage", sideCloseStage);
        poseMap.put("Amp Direct", ampDirect);
        poseMap.put("Left Side Far Stage", leftSideFarStage);

        // Initialize variables to keep track of the shortest distance and the
        // corresponding Pose2d object
        double shortestDistance = Double.MAX_VALUE;
        String closestKey = null;

        // Iterate through the HashMap entries
        for (HashMap.Entry<String, List<Pose2d>> entry : poseMap.entrySet()) {

            List<Pose2d> poses = new ArrayList<>(entry.getValue());

            if (poses.size() == 0) {
                continue;
            }

            double distance = 0;

            Pose2d firstPose = currentPose;
            Pose2d secondPose = poses.get(0);

            distance = firstPose.getTranslation().getDistance(secondPose.getTranslation());

            if (distance < shortestDistance) {
                shortestDistance = distance;
                closestKey = entry.getKey();
            }
        }

        return poseMap.get(closestKey);

    }
}
