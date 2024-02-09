// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.drivetrain;

import java.util.ArrayList;
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
    static Translation2d centerWaypoint = new Translation2d(8.641554, 4.082977);

    public static List<Pose2d> getShortestTrajec(SwerveDriveSubsystem drive){
        // System.out.println(drive.getPose());
        Translation2d currentTranslation = drive.getPose().getTranslation();
        
        double length = currentTranslation.getDistance(waypoint);
        double length1 = currentTranslation.getDistance(waypoint2);

        if(length > length1){
            List<Pose2d> pose = new ArrayList<>();
            // pose.add(new Pose2d(waypoint2, new Rotation2d()));
            // pose.add(new Pose2d(centerWaypoint, new Rotation2d()));
            TrajectoryList trajecList = JSONParser.getTrajectoryList("src/main/deploy/choreo/centerAmp.traj");
            trajecList.removeLastIndex();
            pose.addAll(trajecList.getPoseArray());
            return pose;
        } else{
            List<Pose2d> pose = new ArrayList<>();
            pose.add(new Pose2d(waypoint, new Rotation2d()));
            TrajectoryList trajecList = JSONParser.getTrajectoryList("src/main/deploy/choreo/sideAmp.traj");
            trajecList.removeLastIndex();
            pose.addAll(trajecList.getPoseArray());
            
            return pose;
        }

       

       
    }
}
