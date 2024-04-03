package org.team100.lib.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.copies.TrajectoryConfig100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.StraightLineTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;

class StraightLineTrajectoryTest {
    private static final double kDelta = 0.001;

    @Test
    void testRestToRest() {
        TrajectoryConfig100 c = SwerveKinodynamicsFactory.get().newTrajectoryConfig(2, 2);
        assertEquals(0, c.getStartVelocity(), kDelta);
        StraightLineTrajectory t = new StraightLineTrajectory(c);
        SwerveState start = new SwerveState(GeometryUtil.kPoseZero, GeometryUtil.kTwist2dIdentity);
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory traj = t.apply(start, end);
        assertEquals(1.414, traj.getTotalTimeSeconds(), kDelta);
    }

    @Test
    void testMovingToRest() {
        Experiments.instance.testOverride(Experiment.UseInitialVelocity, true);
        TrajectoryConfig100 c = SwerveKinodynamicsFactory.get().newTrajectoryConfig(2, 2);
        assertEquals(0, c.getStartVelocity(), kDelta);
        StraightLineTrajectory t = new StraightLineTrajectory(c);
        SwerveState start = new SwerveState(GeometryUtil.kPoseZero, new Twist2d(1, 0, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory traj = t.apply(start, end);
        assertEquals(1.082, traj.getTotalTimeSeconds(), kDelta);
    }

    @Test
    void testBackingUp() {
        Experiments.instance.testOverride(Experiment.UseInitialVelocity, true);
        TrajectoryConfig100 c = SwerveKinodynamicsFactory.get().newTrajectoryConfig(2, 2);
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
        Experiments.instance.testOverride(Experiment.UseInitialVelocity, true);
        TrajectoryConfig100 c = SwerveKinodynamicsFactory.get().newTrajectoryConfig(2, 2);
        assertEquals(0, c.getStartVelocity(), kDelta);
        StraightLineTrajectory t = new StraightLineTrajectory(c);
        SwerveState start = new SwerveState(GeometryUtil.kPoseZero, new Twist2d(0, 1, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory traj = t.apply(start, end);
        assertEquals(1.452, traj.getTotalTimeSeconds(), kDelta);
    }
}
