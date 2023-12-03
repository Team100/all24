package org.team100.lib.commands.drivetrain;

import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.controller.DriveControllersFactory;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.trajectory.StraightLineTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;

public class Line {

    public static Command line(Pose2d goal, SwerveDriveSubsystem drivetrain, TrajectoryConfig config) {
        StraightLineTrajectory maker = new StraightLineTrajectory(config);
        Identity identity = Identity.get();
        DriveControllers controllers = new DriveControllersFactory().get(identity);
        HolonomicDriveController3 controller = new HolonomicDriveController3(controllers);
        controller.setTolerance(0.1, 0.1, 0.1, 0.1);
        return new DriveToWaypoint3(goal, drivetrain, maker, controller);
    }
}
