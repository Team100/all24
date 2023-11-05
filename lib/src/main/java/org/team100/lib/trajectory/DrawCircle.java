// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.trajectory;

import org.team100.lib.autonomous.DriveToWaypoint3;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrawCircle extends SequentialCommandGroup {
  /** Creates a new DrawCircle. */
  public DrawCircle(Pose2d[] goalArr, SwerveDriveSubsystem drivetrain, SwerveDriveKinematics kinematics) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new DriveToWaypoint3(goalArr[0], drivetrain, kinematics),
    new DriveToWaypoint3(goalArr[1], drivetrain, kinematics),
    new DriveToWaypoint3(goalArr[2], drivetrain, kinematics),
    new DriveToWaypoint3(goalArr[3], drivetrain, kinematics),
    new DriveToWaypoint3(goalArr[4], drivetrain, kinematics)



    

        
    );
  }
}
