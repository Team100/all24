package org.team100.lib.commands.drivetrain;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DrivePIDFController;
import org.team100.lib.controller.DrivePursuitController;
import org.team100.lib.controller.DriveRamseteController;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.Fixture;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Pose2d;

class DriveToWaypoint100Test {
    Fixture fixture = new Fixture();
    
    @Test
    void testWithPID() {
        Pose2d goal = GeometryUtil.kPoseZero;
        SwerveDriveSubsystem drivetrain = fixture.drive;
        TrajectoryPlanner planner = new TrajectoryPlanner(fixture.swerveKinodynamics);
        DriveMotionController controller = new DrivePIDFController(false);
        DriveToWaypoint100 command = new DriveToWaypoint100(goal, drivetrain, planner, controller, fixture.swerveKinodynamics);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }

    @Test
    void testWithPursuit() {
        Pose2d goal = GeometryUtil.kPoseZero;
        SwerveDriveSubsystem drivetrain = fixture.drive;
        TrajectoryPlanner planner = new TrajectoryPlanner(fixture.swerveKinodynamics);
        DriveMotionController controller = new DrivePursuitController();
        DriveToWaypoint100 command = new DriveToWaypoint100(goal, drivetrain, planner, controller, fixture.swerveKinodynamics);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }

    @Test
    void testWithRamsete() {
        Pose2d goal = GeometryUtil.kPoseZero;
        SwerveDriveSubsystem drivetrain = fixture.drive;
        TrajectoryPlanner planner = new TrajectoryPlanner(fixture.swerveKinodynamics);
        DriveMotionController controller = new DriveRamseteController();
        DriveToWaypoint100 command = new DriveToWaypoint100(goal, drivetrain, planner, controller, fixture.swerveKinodynamics);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }

    @Test
    void testWithFF() {
        Pose2d goal = GeometryUtil.kPoseZero;
        SwerveDriveSubsystem drivetrain = fixture.drive;
        TrajectoryPlanner planner = new TrajectoryPlanner(fixture.swerveKinodynamics);
        DriveMotionController controller = new DrivePIDFController(true);
        DriveToWaypoint100 command = new DriveToWaypoint100(goal, drivetrain, planner, controller, fixture.swerveKinodynamics);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }
}
