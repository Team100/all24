// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.frc2024.motion.drivetrain.manual.ManualWithShooterLock;
import org.team100.lib.commands.drivetrain.DriveBackwards;
import org.team100.lib.commands.drivetrain.DriveManually;
import org.team100.lib.commands.drivetrain.DriveSimple;
import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
import org.team100.lib.commands.drivetrain.DriveWithTrajectory;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrimitiveAuto extends SequentialCommandGroup {
  /** Creates a new PrimitiveAuto. */
  public PrimitiveAuto(SwerveDriveSubsystem m_drive, ManualWithShooterLock shooterLock, TrajectoryPlanner planner, DriveMotionController pidfController, DriveMotionController ppController, SwerveKinodynamics limits) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Pose2d goal = new Pose2d(2.348966, 6.646538, new Rotation2d(Math.PI));
    Pose2d goal2 = new Pose2d(2.228950, 5.510457, new Rotation2d(Math.PI));
    Pose2d goal3 = new Pose2d(2.393838, 4.552123, new Rotation2d(Math.PI));
    Pose2d goal4 = new Pose2d(3.250814, 3.176323, new Rotation2d(Math.PI));

    

    addCommands(
        new DriveToWaypoint100(goal, m_drive, planner, pidfController, limits, true),
        new ParallelDeadlineGroup(new DriveBackwards(m_drive, 0.05), new IntakeWithSensor()),
        // new DriveSimple(m_drive, shooterLock),
        new WaitCommand(1),
        new DriveToWaypoint100(goal2, m_drive, planner, pidfController, limits, true),
        new ParallelDeadlineGroup(new DriveBackwards(m_drive, 0.05), new IntakeWithSensor()),
        // new DriveSimple(m_drive, shooterLock),
        new WaitCommand(1),
        new DriveToWaypoint100(goal3, m_drive, planner, pidfController, limits, true),
        new ParallelDeadlineGroup(new DriveBackwards(m_drive, 0.05), new IntakeWithSensor()),
        // new DriveSimple(m_drive, shooterLock),
        new DriveWithTrajectory(m_drive, planner, pidfController, limits, "src/main/deploy/choreo/Note3to4.traj"),
        // new ParallelDeadlineGroup(new DriveBackwards(m_drive, 0.05), new IntakeWithSensor()),
        new DriveToWaypoint100(goal4, m_drive, planner, pidfController, limits, true)


        
    );
  }
}
