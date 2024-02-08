// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
import org.team100.lib.commands.drivetrain.DriveWithWaypoints;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Amp extends SequentialCommandGroup {
  /** Creates a new Amp. */
  public Amp(Supplier<Pose2d> poseSupplier, SwerveDriveSubsystem m_swerve, TrajectoryPlanner planner, DriveMotionController controller, SwerveKinodynamics limits) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    List<Pose2d> waypoints = new ArrayList<>();
    List<Rotation2d> headings = new ArrayList<>();

    // waypoints.add(poseSupplier.get());
    // headings.add(poseSupplier.get().getRotation());

    // waypoints.add(new Pose2d(10.701702, 1.557158, new Rotation2d()));
    waypoints.add(new Pose2d(7.715937, 6.669663, new Rotation2d()));
    // waypoints.add(new Pose2d(1.892994, 7.747878, new Rotation2d()));


    headings.add(new Rotation2d());
    // headings.add(new Rotation2d());
    // headings.add(new Rotation2d());

    
    addCommands(
        new DriveWithWaypoints(m_swerve, planner, controller, limits, new Pose2d(7.715937, 6.669663, new Rotation2d(0)))

        // new DriveToWaypoint100(new Pose2d(10.701702, 1.557158, new Rotation2d(0)), m_swerve, planner, controller, limits),
        // new DriveToWaypoint100(new Pose2d(7.715937, 6.669663, new Rotation2d(0)), m_swerve, planner, controller, limits)
        // new DriveToWaypoint100(new Pose2d(1.892994, 7.747878, new Rotation2d(0)), m_swerve, planner, controller, limits)

        //     waypoints.add(new Pose2d(10.701702, 1.557158, new Rotation2d()));
        //     headings.add(new Rotation2d());
    
        new DriveWithWaypoints(m_swerve, planner, controller, limits, waypoints, headings)

        // new DriveToWaypoint100(new Pose2d(10.701702, 1.557158, new Rotation2d(0)), m_swerve, planner, controller, limits)
    );
  }
}
