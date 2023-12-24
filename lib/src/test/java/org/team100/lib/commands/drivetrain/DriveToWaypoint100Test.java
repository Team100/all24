package org.team100.lib.commands.drivetrain;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DrivePIDFController;
import org.team100.lib.controller.DrivePursuitController;
import org.team100.lib.controller.DriveRamseteController;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.MockSwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinematics.SwerveDriveKinematicsFactory;
import org.team100.lib.swerve.SwerveKinematicLimits;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

class DriveToWaypoint100Test {
    private static final SwerveDriveKinematics kinematics = SwerveDriveKinematicsFactory.get(0.2,0.2);
    

    @Test
    void testWithPID() {
        Pose2d goal = GeometryUtil.kPoseZero;
        MockSwerveDriveSubsystem drivetrain = new MockSwerveDriveSubsystem();
        SwerveKinematicLimits limits = new SwerveKinematicLimits(4, 2, 2, 10, 7);
        TrajectoryPlanner planner = new TrajectoryPlanner(kinematics, limits);
        DriveMotionController controller = new DrivePIDFController(false);
        DriveToWaypoint100 command = new DriveToWaypoint100(goal, drivetrain, planner, controller);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }

    @Test
    void testWithPursuit() {
        Pose2d goal = GeometryUtil.kPoseZero;
        MockSwerveDriveSubsystem drivetrain = new MockSwerveDriveSubsystem();
        SwerveKinematicLimits limits = new SwerveKinematicLimits(4, 2, 2, 10, 7);
        TrajectoryPlanner planner = new TrajectoryPlanner(kinematics, limits);
        DriveMotionController controller = new DrivePursuitController();
        DriveToWaypoint100 command = new DriveToWaypoint100(goal, drivetrain, planner, controller);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }

    @Test
    void testWithRamsete() {
        Pose2d goal = GeometryUtil.kPoseZero;
        MockSwerveDriveSubsystem drivetrain = new MockSwerveDriveSubsystem();
        SwerveKinematicLimits limits = new SwerveKinematicLimits(4, 2, 2, 10, 7);
        TrajectoryPlanner planner = new TrajectoryPlanner(kinematics, limits);
        DriveMotionController controller = new DriveRamseteController();
        DriveToWaypoint100 command = new DriveToWaypoint100(goal, drivetrain, planner, controller);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }

    @Test
    void testWithFF() {
        Pose2d goal = GeometryUtil.kPoseZero;
        MockSwerveDriveSubsystem drivetrain = new MockSwerveDriveSubsystem();
        SwerveKinematicLimits limits = new SwerveKinematicLimits(4, 2, 2, 10, 7);
        TrajectoryPlanner planner = new TrajectoryPlanner(kinematics, limits);
        DriveMotionController controller = new DrivePIDFController(true);
        DriveToWaypoint100 command = new DriveToWaypoint100(goal, drivetrain, planner, controller);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }
}
