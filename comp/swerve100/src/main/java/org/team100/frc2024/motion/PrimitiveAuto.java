package org.team100.frc2024.motion;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.frc2024.motion.drivetrain.manual.ManualWithShooterLock;
import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
import org.team100.lib.commands.drivetrain.DriveToWaypoint3;
import org.team100.lib.commands.drivetrain.DriveWithTrajectory;
import org.team100.lib.commands.drivetrain.Rotate;
import org.team100.lib.commands.drivetrain.RotateTo180;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class PrimitiveAuto extends SequentialCommandGroup {
  /** Creates a new PrimitiveAuto. */
  public PrimitiveAuto(SwerveDriveSubsystem m_drive, ManualWithShooterLock shooterLock, TrajectoryPlanner planner, DriveMotionController pidfController, DriveMotionController ppController, SwerveKinodynamics limits, HeadingInterface heading) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Pose2d goal = new Pose2d(2.4, 4.5, new Rotation2d(0));
    // Pose2d goal2 = new Pose2d(2.228950, 5.510457, new Rotation2d(Math.PI));
    // Pose2d goal3 = new Pose2d(2.393838, 4.552123, new Rotation2d(Math.PI));
    // Pose2d goal4 = new Pose2d(3.250814, 3.176323, new Rotation2d(Math.PI));
    // TrajectoryConfig config = new TrajectoryConfig(4, 5);
    // StraightLineTrajectory maker = new StraightLineTrajectory(config);

    Pose2d goal = new Pose2d(2.29, 1.06, new Rotation2d());
    Pose2d goal2 = new Pose2d(2.34, 2.85, new Rotation2d());
    Pose2d goal3 = new Pose2d(2.52, 1.06, new Rotation2d());


    

    addCommands(
        new DriveToWaypoint100(goal, m_drive, planner, pidfController, limits, () -> ShooterUtil.getRobotRotationToSpeaker(goal.getTranslation(), 0.25))
        // new DriveToWaypoint3(goal, m_drive, maker, new HolonomicDriveController3())
        // new ParallelDeadlineGroup(new DriveBackwards(m_drive, 0.05), new IntakeWithSensor()),
        // new DriveSimple(m_drive, shooterLock),
        // new WaitCommand(1),
        // new Rotate(m_drive, heading, limits, 0)
        // new DriveToWaypoint100(goal2, m_drive, planner, pidfController, limits, () -> ShooterUtil.getRobotRotationToSpeaker(goal2.getTranslation(), 0.25))
        // new ParallelDeadlineGroup(new DriveBackwards(m_drive, 0.05), new IntakeWithSensor()),
        // new DriveSimple(m_drive, shooterLock),
        // new Rotate(m_drive, heading, limits, 0),

        // new WaitCommand(1)
        // new DriveToWaypoint100(goal3, m_drive, planner, pidfController, limits, true),
        // // new ParallelDeadlineGroup(new DriveBackwards(m_drive, 0.05), new IntakeWithSensor()),
        // new DriveSimple(m_drive, shooterLock)
        // new DriveWithTrajectory(m_drive, planner, pidfController, limits, "src/main/deploy/choreo/Note3to4.traj"),
        // // new ParallelDeadlineGroup(new DriveBackwards(m_drive, 0.05), new IntakeWithSensor()),
        // new DriveToWaypoint100(goal4, m_drive, planner, pidfController, limits, true)
        // new Rotate(m_drive, heading, limits, 0)


        
    );
  }
}
