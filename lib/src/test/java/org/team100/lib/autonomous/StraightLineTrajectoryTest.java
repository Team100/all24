package org.team100.lib.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.trajectory.StraightLineTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;

public class StraightLineTrajectoryTest {
    private static final double kDelta = 0.001;

    @Test
    void testSimple() {
        SwerveDriveKinematics k = new SwerveDriveKinematics(
                new Translation2d(0.1, 0.1),
                new Translation2d(0.1, -0.1),
                new Translation2d(-0.1, 0.1),
                new Translation2d(-0.1, -0.1)

        );
        StraightLineTrajectory t = new StraightLineTrajectory(k);
        Pose2d start = new Pose2d();
        Pose2d end = new Pose2d(1, 0, new Rotation2d());
        Trajectory traj = t.apply(start, end);
        // System.out.println(traj);
        assertEquals(1.414, traj.getTotalTimeSeconds(), kDelta);
    }
}
