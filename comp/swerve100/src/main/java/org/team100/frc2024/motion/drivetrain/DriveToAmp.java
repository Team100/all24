// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.drivetrain;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.team100.lib.commands.drivetrain.JSONParser;
import org.team100.lib.commands.drivetrain.TrajectoryList;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class DriveToAmp {

    static Translation2d waypoint = new Translation2d(10.320774, 2.098546);
    static Translation2d waypoint2 = new Translation2d(11.693578, 4.055302);


    public static List<Pose2d> getShortestTrajec(SwerveDriveSubsystem drive){
        // System.out.println(drive.getPose());
        Translation2d currentTranslation = drive.getPose().getTranslation();
        
        double length = currentTranslation.getDistance(waypoint);
        double length1 = currentTranslation.getDistance(waypoint2);

        if(length > length1){
            List<Pose2d> pose = new ArrayList<>();
            pose.add(new Pose2d(waypoint2, new Rotation2d()));
            // pose.add(new Pose2d(centerWaypoint, new Rotation2d()));
            // TrajectoryList trajecList = JSONParser.getTrajectoryList("src/main/deploy/choreo/centerAmp.traj");
            // trajecList.removeLastIndex();
            // pose.addAll(trajecList.getPoseArray());
            pose.add(new Pose2d(7.682685, 6.063078, new Rotation2d()));
            pose.add(new Pose2d(1.715115, 7.334519, new Rotation2d()));

            return pose;
        } else{
            List<Pose2d> pose = new ArrayList<>();
            pose.add(new Pose2d(waypoint, new Rotation2d()));
            // TrajectoryList trajecList = JSONParser.getTrajectoryList("src/main/deploy/choreo/sideAmp.traj");
            // trajecList.removeLastIndex();
            // pose.addAll(trajecList.getPoseArray());
            pose.add(new Pose2d(7.682685, 6.063078, new Rotation2d()));
            pose.add(new Pose2d(1.715115, 7.334519, new Rotation2d()));
            
            return pose;
        }

       

       
    }

    public static List<Pose2d> getShortestTrajecNew(SwerveDriveSubsystem drive){
        HashMap<String, Pose2d> poseMap = new HashMap<>();
        Translation2d currentTranslation = drive.getPose().getTranslation();

        
        
        poseMap.put("Side Far Stage", new Pose2d(10.320774, 2.098546, new Rotation2d()));
        poseMap.put("Center Far Stage", new Pose2d(11.693578, 4.055302, new Rotation2d()));
        poseMap.put("Center Stage", new Pose2d(5.885868, 6.359865, new Rotation2d()));

        // Initialize variables to keep track of the shortest distance and the corresponding Pose2d object
        double shortestDistance = Double.MAX_VALUE;
        Pose2d closestPose = null;
        String closestKey = null;

        // Iterate through the HashMap entries
        for (HashMap.Entry<String, Pose2d> entry : poseMap.entrySet()) {
            Pose2d pose = entry.getValue();
            double distance = currentTranslation.getDistance(pose.getTranslation());
            if (distance < shortestDistance) {
                shortestDistance = distance;
                closestPose = pose;
                closestKey = entry.getKey();
            }
        }

        List<Pose2d> completeWaypoints = new ArrayList<>();

        if(closestKey == "Side Far Stage"){
            completeWaypoints.add(closestPose);
            completeWaypoints.add(new Pose2d(7.682685, 6.063078, new Rotation2d()));
        } else if(closestKey == "Center Far Stage"){
            completeWaypoints.add(closestPose);
            completeWaypoints.add(new Pose2d(7.682685, 6.063078, new Rotation2d()));
        } else {
            completeWaypoints.add(closestPose);
        }

        completeWaypoints.add(new Pose2d(1.715115, 7.334519, new Rotation2d()));

        return completeWaypoints;


    }
}
