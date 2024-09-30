package org.team100.lib.commands.drivetrain.for_testing;

import java.util.List;

import org.team100.lib.commands.drivetrain.DriveToWaypoint3;
import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.timing.ConstantConstraint;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.TrajectoryMaker;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Drive in a square. This used to be called "DrawCircle."
 */
public class DrawSquare extends SequentialCommandGroup implements Glassy {
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
            SupplierLogger2 parent,
            SwerveDriveSubsystem drivetrain,
            HolonomicFieldRelativeController controller,
            TrajectoryVisualization viz) {
        DriveToWaypoint3.Log log = new DriveToWaypoint3.Log(parent.child(this));
        TrajectoryMaker tmaker = new TrajectoryMaker(List.of(new ConstantConstraint(maxVelocityM_S, maxAccelM_S_S)));
        StraightLineTrajectory maker = new StraightLineTrajectory(true, tmaker);
        addCommands(
                new DriveToWaypoint3(log,
                        new Pose2d(-0.5, -0.5, GeometryUtil.kRotationZero),
                        drivetrain, maker,
                        controller, viz),
                new DriveToWaypoint3(log,
                        new Pose2d(-0.5, 0.5, GeometryUtil.kRotationZero),
                        drivetrain, maker,
                        controller, viz),
                new DriveToWaypoint3(log,
                        new Pose2d(0.5, 0.5, GeometryUtil.kRotationZero),
                        drivetrain, maker,
                        controller, viz),
                new DriveToWaypoint3(log,
                        new Pose2d(0.5, -0.5, GeometryUtil.kRotationZero),
                        drivetrain, maker,
                        controller, viz),
                new DriveToWaypoint3(log,
                        new Pose2d(-0.5, -0.5, GeometryUtil.kRotationZero),
                        drivetrain, maker,
                        controller, viz)

        );
    }
}
