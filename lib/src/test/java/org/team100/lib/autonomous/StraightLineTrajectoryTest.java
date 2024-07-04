package org.team100.lib.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryMaker;

import edu.wpi.first.math.geometry.Pose2d;

class StraightLineTrajectoryTest {
    private static final double kDelta = 0.001;
    Logger logger = Telemetry.get().testLogger();
    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get(logger);
    List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
    TrajectoryMaker maker = new TrajectoryMaker(constraints);

    @Test
    void testRestToRest() {
        StraightLineTrajectory t = new StraightLineTrajectory(maker);
        SwerveState start = new SwerveState(GeometryUtil.kPoseZero, new FieldRelativeVelocity(0, 0, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory100 traj = t.apply(start, end);
        assertEquals(1, traj.getTotalTimeSeconds(), kDelta);
    }

    @Test
    void testMovingToRest() {
        Experiments.instance.testOverride(Experiment.UseInitialVelocity, true);
        StraightLineTrajectory t = new StraightLineTrajectory(maker);
        SwerveState start = new SwerveState(GeometryUtil.kPoseZero, new FieldRelativeVelocity(1, 0, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory100 traj = t.apply(start, end);
        assertEquals(1.5, traj.getTotalTimeSeconds(), kDelta);
    }

    @Test
    void testBackingUp() {
        Experiments.instance.testOverride(Experiment.UseInitialVelocity, true);
        StraightLineTrajectory t = new StraightLineTrajectory(maker);
        SwerveState start = new SwerveState(GeometryUtil.kPoseZero, new FieldRelativeVelocity(-1, 0, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory100 traj = t.apply(start, end);
        assertEquals(1.824, traj.getTotalTimeSeconds(), kDelta);
    }

    @Test
    void test2d() {
        Experiments.instance.testOverride(Experiment.UseInitialVelocity, true);
        StraightLineTrajectory t = new StraightLineTrajectory(maker);
        SwerveState start = new SwerveState(GeometryUtil.kPoseZero, new FieldRelativeVelocity(0, 1, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory100 traj = t.apply(start, end);
        assertEquals(2.109, traj.getTotalTimeSeconds(), kDelta);
    }
}
