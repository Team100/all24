// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.amp;

import org.team100.lib.commands.drivetrain.DriveToState101;
import org.team100.lib.commands.drivetrain.DriveWithProfileNote;
import org.team100.lib.commands.drivetrain.DriveWithProfile2;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.HolonomicDriveController100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToAmp extends SequentialCommandGroup {
  /** Creates a new DriveToAmp. */
  public DriveToAmp(SwerveDriveSubsystem drive, SwerveKinodynamics limits, TrajectoryPlanner planner, DriveMotionController controller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new DriveToState101(new Pose2d(5.765933, 6.412866, new Rotation2d(Math.PI/2)), new Twist2d(-3, 3, 3 * Math.PI / 4), drive, planner, controller, limits),
        new DriveWithProfile2( ()-> new Pose2d(1.834296, 7.474794, new Rotation2d(Math.PI/2)), drive, new HolonomicDriveController100(), limits),
        new DriveWithProfile2( ()-> new Pose2d(1.834296, 7.799454, new Rotation2d(Math.PI/2)), drive, new HolonomicDriveController100(), limits)

    );
  }
}
