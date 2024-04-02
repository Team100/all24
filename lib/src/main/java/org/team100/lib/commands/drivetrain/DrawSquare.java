package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.copies.TrajectoryConfig100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.StraightLineTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Drive in a square.  This used to be called "DrawCircle."
 */
public class DrawSquare extends SequentialCommandGroup {
    private static final double maxVelocityM_S = 2.0;
    private static final double maxAccelM_S_S = 2;

    /**
     * Draw a square like so:
     * 
     * .........X
     * ..........
     * .....3---4
     * .....|...|
     * .Y...2---1/5
     */
    public DrawSquare(
            SwerveDriveSubsystem drivetrain,
            SwerveKinodynamics kinodynamics,
            HolonomicDriveController3 controller) {
        TrajectoryConfig100 config = kinodynamics.newTrajectoryConfig(maxVelocityM_S, maxAccelM_S_S);
        StraightLineTrajectory maker = new StraightLineTrajectory(config);
        addCommands(
                new DriveToWaypoint3(new Pose2d(-0.5, -0.5, GeometryUtil.kRotationZero), drivetrain, maker, controller),
                new DriveToWaypoint3(new Pose2d(-0.5, 0.5, GeometryUtil.kRotationZero), drivetrain, maker, controller),
                new DriveToWaypoint3(new Pose2d(0.5, 0.5, GeometryUtil.kRotationZero), drivetrain, maker, controller),
                new DriveToWaypoint3(new Pose2d(0.5, -0.5, GeometryUtil.kRotationZero), drivetrain, maker, controller),
                new DriveToWaypoint3(new Pose2d(-0.5, -0.5, GeometryUtil.kRotationZero), drivetrain, maker, controller)

        );
    }
}
