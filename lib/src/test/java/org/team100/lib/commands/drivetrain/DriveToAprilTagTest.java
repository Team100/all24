package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.Target;
import org.team100.lib.motion.drivetrain.MockSwerveDriveSubsystem;
import org.team100.lib.trajectory.StraightLineTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

/** Exercise the code. */
class DriveToAprilTagTest {

    @Test
    void testSimple() throws IOException {
        MockSwerveDriveSubsystem drivetrain = new MockSwerveDriveSubsystem();
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(0.1, 0.1),
                new Translation2d(0.1, -0.1),
                new Translation2d(-0.1, 0.1),
                new Translation2d(-0.1, -0.1));
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .blueLayout("2023-chargedup.json");

        TrajectoryConfig config = new TrajectoryConfig(4, 2).setKinematics(kinematics);

        StraightLineTrajectory maker = new StraightLineTrajectory(config);
        Transform2d transform = new Transform2d(
                new Translation2d(-1, -1),
                GeometryUtil.kRotationZero);

        Optional<Pose2d> goal = Target.goal(layout, 1, transform);
        assertTrue(goal.isPresent());
        DriveToAprilTag command = new DriveToAprilTag(goal.get(), drivetrain, maker);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }
}
