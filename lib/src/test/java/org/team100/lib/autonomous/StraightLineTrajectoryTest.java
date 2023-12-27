package org.team100.lib.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.StraightLineTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

class StraightLineTrajectoryTest {
    private static final double kDelta = 0.001;

    @Test
    void testRestToRest() {
        SwerveDriveKinematics k = SwerveKinodynamicsFactory.get().getKinematics();
        TrajectoryConfig c = new TrajectoryConfig(2, 2).setKinematics(k);
        assertEquals(0, c.getStartVelocity(), kDelta);
        StraightLineTrajectory t = new StraightLineTrajectory(c);
        SwerveState start = new SwerveState(GeometryUtil.kPoseZero, GeometryUtil.kTwist2dIdentity);
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory traj = t.apply(start, end);
        assertEquals(1.414, traj.getTotalTimeSeconds(), kDelta);
    }

    @Test
    void testMovingToRest() {
        SwerveDriveKinematics k = SwerveKinodynamicsFactory.get().getKinematics();
        TrajectoryConfig c = new TrajectoryConfig(2, 2).setKinematics(k);
        assertEquals(0, c.getStartVelocity(), kDelta);
        StraightLineTrajectory t = new StraightLineTrajectory(c);
        SwerveState start = new SwerveState(GeometryUtil.kPoseZero, new Twist2d(1, 0, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory traj = t.apply(start, end);
        assertEquals(1.082, traj.getTotalTimeSeconds(), kDelta);
    }

    @Test
    void testBackingUp() {
        SwerveDriveKinematics k = SwerveKinodynamicsFactory.get().getKinematics();
        TrajectoryConfig c = new TrajectoryConfig(2, 2).setKinematics(k);
        assertEquals(0, c.getStartVelocity(), kDelta);
        StraightLineTrajectory t = new StraightLineTrajectory(c);
        SwerveState start = new SwerveState(GeometryUtil.kPoseZero, new Twist2d(-1, 0, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory traj = t.apply(start, end);
        // it can't do this because the spline generator gets confused
        assertEquals(0, traj.getTotalTimeSeconds(), kDelta);
    }

    @Test
    void test2d() {
        SwerveDriveKinematics k = SwerveKinodynamicsFactory.get().getKinematics();
        TrajectoryConfig c = new TrajectoryConfig(2, 2).setKinematics(k);
        assertEquals(0, c.getStartVelocity(), kDelta);
        StraightLineTrajectory t = new StraightLineTrajectory(c);
        SwerveState start = new SwerveState(GeometryUtil.kPoseZero, new Twist2d(0, 1, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory traj = t.apply(start, end);
        assertEquals(1.452, traj.getTotalTimeSeconds(), kDelta);
    }
}
