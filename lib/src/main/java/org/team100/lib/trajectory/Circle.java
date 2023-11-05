// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.trajectory;

import org.team100.lib.autonomous.DriveToWaypoint3;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;

public class Circle extends Command {
  SwerveDriveSubsystem m_drivetrain;
  SwerveDriveKinematics m_kinematics;
  DriveToWaypoint3 circle;
  /** Creates a new Circle. */
  public Circle(Pose2d goal, SwerveDriveSubsystem drivetrain, SwerveDriveKinematics kinematics) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drivetrain = drivetrain;
    m_kinematics = kinematics;


    circle = new DriveToWaypoint3(goal, m_drivetrain, m_kinematics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    circle.initialize();


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    circle.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
