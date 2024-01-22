package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.Target;
import org.team100.lib.motion.drivetrain.Fixture;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.StraightLineTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

class DriveToWaypoint3Test {
    private static final double kDelta = 0.001;

    private final Fixture fixture = new Fixture();

    @AfterEach
    void close() {
        fixture.close();
    }

    @Test
    void testSimple() {
        Pose2d goal = GeometryUtil.kPoseZero;
        SwerveDriveSubsystem drivetrain = fixture.drive;

        StraightLineTrajectory trajectories = new StraightLineTrajectory(null) {
            public Trajectory apply(SwerveState startState, Pose2d end) {
                return new Trajectory(List.of(new Trajectory.State()));
            }
        };

        HolonomicDriveController3 controller = new HolonomicDriveController3();

        DriveToWaypoint3 command = new DriveToWaypoint3(
                goal,
                drivetrain,
                trajectories,
                controller);

        command.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), kDelta);
        command.execute();
        command.end(false);
    }

    /** Demonstrate how to use DriveToWaypoint to go to apriltags. */
    @Test
    void testAprilTag() throws IOException {
        SwerveDriveSubsystem drivetrain = fixture.drive;
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .blueLayout("2023-chargedup.json");

        TrajectoryConfig config = SwerveKinodynamicsFactory.get().newTrajectoryConfig(4, 2);

        StraightLineTrajectory maker = new StraightLineTrajectory(config);
        Transform2d transform = new Transform2d(
                new Translation2d(-1, -1),
                GeometryUtil.kRotationZero);

        Optional<Pose2d> goal = Target.goal(layout, 1, transform);
        assertTrue(goal.isPresent());

        HolonomicDriveController3 m_controller = new HolonomicDriveController3();

        DriveToWaypoint3 command = new DriveToWaypoint3(goal.get(), drivetrain, maker, m_controller);

        command.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), kDelta);
        command.execute();
        command.end(false);
    }

}
