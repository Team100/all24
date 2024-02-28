package org.team100.frc2024;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.team100.frc2024.motion.AmpUtil;
import org.team100.lib.commands.drivetrain.DriveWithWaypoints;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Amp extends SequentialCommandGroup {
    public Amp(
            Supplier<Pose2d> poseSupplier,
            SwerveDriveSubsystem m_swerve,
            TrajectoryPlanner planner,
            DriveMotionController controller,
            SwerveKinodynamics limits) {

        List<Pose2d> waypoint = new ArrayList<>();
        waypoint.add(new Pose2d(11.266917, 1.374847, new Rotation2d(0)));
        waypoint.add(new Pose2d(8.249782, 4.167293, new Rotation2d(0)));
        waypoint.add(new Pose2d(4.763524, 6.737026, new Rotation2d(0)));

        addCommands(
                // new DriveWithWaypoints(m_swerve, planner, controller, limits, waypoint)
                new DriveWithWaypoints(m_swerve, planner, controller, limits,
                        () -> AmpUtil.getShortestTrajecNew(m_swerve))

        // new DriveWithTrajectory(m_swerve, planner, controller, limits,
        // "src/main/deploy/choreo/Note3to4.traj")
        // new DriveToWaypoint100(new Pose2d(10.701702, 1.557158, new Rotation2d(0)),
        // m_swerve, planner, controller, limits),
        // new DriveToWaypoint100(new Pose2d(7.715937, 6.669663, new Rotation2d(0)),
        // m_swerve, planner, controller, limits)
        // new DriveToWaypoint100(new Pose2d(1.892994, 7.747878, new Rotation2d(0)),
        // m_swerve, planner, controller, limits)

        // waypoints.add(new Pose2d(10.701702, 1.557158, new Rotation2d()));
        // headings.add(new Rotation2d());

        // new DriveWithWaypoints(m_swerve, planner, controller, limits, waypoints,
        // headings)

        // new DriveToWaypoint100(new Pose2d(10.701702, 1.557158, new Rotation2d(0)),
        // m_swerve, planner, controller, limits)
        );
    }
}
