package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.drivetrain.HolonomicDriveControllerFactory;
import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.Target;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryMaker;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

class DriveToWaypoint3Test extends Fixtured {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TrajectoryVisualization viz = new TrajectoryVisualization(logger);

    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();
    List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
    TrajectoryMaker tmaker = new TrajectoryMaker(constraints);

    @Test
    void testSimple() {
        Pose2d goal = GeometryUtil.kPoseZero;
        SwerveDriveSubsystem drivetrain = fixture.drive;

        StraightLineTrajectory trajectories = new StraightLineTrajectory(true, null) {
            public Trajectory100 apply(SwerveState startState, Pose2d end) {
                return new Trajectory100(List.of(new TimedPose(Pose2dWithMotion.kIdentity, 0, 0, 0)));
            }
        };

        HolonomicFieldRelativeController controller = HolonomicDriveControllerFactory.get(
                new HolonomicFieldRelativeController.Log(logger));
        DriveToWaypoint3.Log log = new DriveToWaypoint3.Log(logger);
        DriveToWaypoint3 command = new DriveToWaypoint3(
                log,
                goal,
                drivetrain,
                trajectories,
                controller,
                viz);
        command.initialize();
        assertEquals(0, fixture.drive.getState().pose().getX(), kDelta);
        command.execute();
        command.end(false);
    }

    /** Demonstrate how to use DriveToWaypoint to go to apriltags. */
    @Test
    void testAprilTag() throws IOException {
        SwerveDriveSubsystem drivetrain = fixture.drive;
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        StraightLineTrajectory maker = new StraightLineTrajectory(true, tmaker);
        Transform2d transform = new Transform2d(
                new Translation2d(-1, -1),
                GeometryUtil.kRotationZero);

        Optional<Pose2d> optGoal = Target.goal(layout, Alliance.Blue, 1, transform);
        assertTrue(optGoal.isPresent());
        Pose2d goal = optGoal.get();
        assertEquals(13.713, goal.getX(), kDelta);
        assertEquals(0.612, goal.getY(), kDelta);
        assertEquals(-1.047, goal.getRotation().getRadians(), kDelta);

        HolonomicFieldRelativeController m_controller = HolonomicDriveControllerFactory.get(
                new HolonomicFieldRelativeController.Log(logger));
        DriveToWaypoint3.Log log = new DriveToWaypoint3.Log(logger);
        DriveToWaypoint3 command = new DriveToWaypoint3(
                log, goal, drivetrain, maker, m_controller, viz);
        command.initialize();
        assertEquals(0, fixture.drive.getState().pose().getX(), kDelta);
        command.execute();
        command.end(false);
    }

}
