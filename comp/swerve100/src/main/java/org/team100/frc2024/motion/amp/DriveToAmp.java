package org.team100.frc2024.motion.amp;

import org.team100.lib.commands.drivetrain.DriveWithProfile2;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.HolonomicDriveController100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveToAmp extends SequentialCommandGroup {
    public DriveToAmp(
            SwerveDriveSubsystem drive,
            SwerveKinodynamics limits,
            TrajectoryPlanner planner,
            DriveMotionController controller) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                // new DriveToState101(new Pose2d(5.765933, 6.412866, new
                // Rotation2d(Math.PI/2)), new Twist2d(-3, 3, 3 * Math.PI / 4), drive, planner,
                // controller, limits),
                new DriveWithProfile2(() -> new Pose2d(1.834296, 7.474794, new Rotation2d(Math.PI / 2)), drive,
                        new HolonomicDriveController100(), limits),
                new DriveWithProfile2(() -> new Pose2d(1.834296, 7.799454, new Rotation2d(Math.PI / 2)), drive,
                        new HolonomicDriveController100(), limits)

        );
    }
}
