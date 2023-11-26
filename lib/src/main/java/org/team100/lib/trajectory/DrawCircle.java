package org.team100.lib.trajectory;

import org.team100.lib.commands.drivetrain.DriveToWaypoint3;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DrawCircle extends SequentialCommandGroup {
    private static final double maxVelocityM_S = 2.0;
    private static final double maxAccelM_S_S = 2;

    public DrawCircle(SwerveDriveSubsystem drivetrain, SwerveDriveKinematics kinematics) {
        TrajectoryConfig config = new TrajectoryConfig(maxVelocityM_S, maxAccelM_S_S).setKinematics(kinematics);
        StraightLineTrajectory maker = new StraightLineTrajectory(config);

        // .........X
        // ..........
        // .....3---4
        // .....|...|
        // .Y...2---1/5

        addCommands(
                new DriveToWaypoint3(new Pose2d(-0.5, -0.5, Rotation2d.fromDegrees(0)), drivetrain, maker),
                new DriveToWaypoint3(new Pose2d(-0.5, 0.5, Rotation2d.fromDegrees(0)), drivetrain, maker),
                new DriveToWaypoint3(new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0)), drivetrain, maker),
                new DriveToWaypoint3(new Pose2d(0.5, -0.5, Rotation2d.fromDegrees(0)), drivetrain, maker),
                new DriveToWaypoint3(new Pose2d(-0.5, -0.5, Rotation2d.fromDegrees(0)), drivetrain, maker)

        );
    }
}
