package org.team100.lib.trajectory;

import org.team100.lib.commands.DriveToWaypoint3;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DrawCircle extends SequentialCommandGroup {

    public DrawCircle(Pose2d[] goals, SwerveDriveSubsystem drivetrain, SwerveDriveKinematics kinematics) {

        StraightLineTrajectory maker = new StraightLineTrajectory(kinematics);
        // addCommands(
        //         new DriveToWaypoint3(goals[0], drivetrain, maker),
        //         new DriveToWaypoint3(goals[1], drivetrain, maker),
        //         new DriveToWaypoint3(goals[2], drivetrain, maker),
        //         new DriveToWaypoint3(goals[3], drivetrain, maker),
        //         new DriveToWaypoint3(goals[4], drivetrain, maker)
        // );

        // new Pose2d(0.5, -0.5, Rotation2d.fromDegrees(180)),
        //         new Pose2d(-0.5, -0.5, Rotation2d.fromDegrees(180)),
        //         new Pose2d(-0.5, 0.5, Rotation2d.fromDegrees(180)),
        //         new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(180))

        addCommands(
            new DriveToWaypoint3(new Pose2d(-0.5, -0.5, Rotation2d.fromDegrees(0)), drivetrain, maker),
            // new DriveToWaypoint3(new Pose2d(-0.5, -0.5, Rotation2d.fromDegrees(0)), drivetrain, maker),
            new DriveToWaypoint3(new Pose2d(-0.5, 0.5, Rotation2d.fromDegrees(0)), drivetrain, maker),
            new DriveToWaypoint3(new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0)), drivetrain, maker),
            new DriveToWaypoint3(new Pose2d(0.5, -0.5, Rotation2d.fromDegrees(0)), drivetrain, maker),
            new DriveToWaypoint3(new Pose2d(-0.5, -0.5, Rotation2d.fromDegrees(0)), drivetrain, maker)

            );
    }
}
