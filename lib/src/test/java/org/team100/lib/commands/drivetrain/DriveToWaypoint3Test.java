package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.BiFunction;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.Target;
import org.team100.lib.motion.drivetrain.MockSwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.StraightLineTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

class DriveToWaypoint3Test {
    @Test
    void testSimple() {
        Pose2d goal = GeometryUtil.kPoseZero;
        MockSwerveDriveSubsystem drivetrain = new MockSwerveDriveSubsystem();
        BiFunction<SwerveState, Pose2d, Trajectory> trajectories = (x, y) -> new Trajectory(List.of(new Trajectory.State()));

        HolonomicDriveController3 controller = new HolonomicDriveController3();

        DriveToWaypoint3 command = new DriveToWaypoint3(
                goal,
                drivetrain,
                trajectories,
                controller);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }

    /** Demonstrate how to use DriveToWaypoing to go to apriltags. */
    @Test
    void testAprilTag() throws IOException {
        MockSwerveDriveSubsystem drivetrain = new MockSwerveDriveSubsystem();
        SwerveDriveKinematics kinematics = SwerveKinodynamicsFactory.get(Identity.BLANK, false).getKinematics();
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .blueLayout("2023-chargedup.json");

        TrajectoryConfig config = new TrajectoryConfig(4, 2).setKinematics(kinematics);

        Experiments e = new Experiments(Identity.BLANK);
        StraightLineTrajectory maker = new StraightLineTrajectory(e, config);
        Transform2d transform = new Transform2d(
                new Translation2d(-1, -1),
                GeometryUtil.kRotationZero);

        Optional<Pose2d> goal = Target.goal(layout, 1, transform);
        assertTrue(goal.isPresent());

        HolonomicDriveController3 m_controller = new HolonomicDriveController3();

        DriveToWaypoint3 command = new DriveToWaypoint3(goal.get(), drivetrain, maker, m_controller);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }

}
