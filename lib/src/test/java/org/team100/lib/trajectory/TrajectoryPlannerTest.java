package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.ConstantConstraint;
import org.team100.lib.timing.SwerveDriveDynamicsConstraint;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.YawRateConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class TrajectoryPlannerTest {
    private static final double kDelta = 0.01;

    /**
     * Stationary trajectories do not work.
     */
    @Test
    void testStationary() {
        List<Pose2d> waypoints = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d());
        List<TimingConstraint> constraints = new ArrayList<>();
        Trajectory100 t = TrajectoryPlanner.restToRest(waypoints, headings, constraints);
        assertTrue(t.isEmpty());
    }

    @Test
    void testLinear() {
        List<Pose2d> waypoints = List.of(new Pose2d(), new Pose2d(1, 0, new Rotation2d()));
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d());
        List<TimingConstraint> constraints = new ArrayList<>();
        Trajectory100 t = TrajectoryPlanner.restToRest(waypoints, headings, constraints);
        assertEquals(80, t.m_points.size());
        TrajectoryPoint p = t.getPoint(40);
        assertEquals(0.5, p.state().state().getPose().getX(), kDelta);
        assertEquals(0, p.state().state().getHeadingRate(), kDelta);
    }

    @Test
    void testBackingUp() {
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, new Rotation2d(Math.PI)),
                new Pose2d(1, 0, GeometryUtil.kRotationZero));
        List<Rotation2d> headings = List.of(
                GeometryUtil.kRotationZero,
                GeometryUtil.kRotationZero);
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.get();

        // these are the same as StraightLineTrajectoryTest.
        List<TimingConstraint> constraints = // new ArrayList<>();
                List.of(
                        new ConstantConstraint(limits.getMaxDriveVelocityM_S(), limits.getMaxDriveAccelerationM_S2()),
                        new SwerveDriveDynamicsConstraint(limits),
                        new YawRateConstraint(limits, 0.2),
                        new CentripetalAccelerationConstraint(limits, 0.2));
        double start_vel = 1;
        double end_vel = 0;
        Trajectory100 t = TrajectoryPlanner.generateTrajectory(
                waypoints,
                headings, constraints, start_vel, end_vel);
        // u-turn trajectories are not allowed.
        assertTrue(t.isEmpty());

    }

    /**
     * Is trajectory planning fast enough to run every loop?
     * This computes a single spline, and makes a schedule along it.
     * 
     * On my desktop machine, that takes about 0.5 ms, or 3% of the budget. On the
     * RIO it's probably several times slower, but still maybe 10% of the budget.
     * so, yeah, you can do a spline every loop.
     */
    @Test
    void testPerformance() {
        List<Pose2d> waypoints = List.of(new Pose2d(), new Pose2d(1, 1, new Rotation2d(Math.PI / 2)));
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d());
        List<TimingConstraint> constraints = new ArrayList<>();
        long startTimeNs = System.nanoTime();
        Trajectory100 t = new Trajectory100();
        final int iterations = 100;
        for (int i = 0; i < iterations; ++i) {
            t = TrajectoryPlanner.restToRest(
                    waypoints, headings, constraints);
        }
        long endTimeNs = System.nanoTime();
        double totalDurationMs = (endTimeNs - startTimeNs) / 1000000.0;
        System.out.printf("total duration ms: %5.3f\n", totalDurationMs);
        System.out.printf("duration per iteration ms: %5.3f\n", totalDurationMs / iterations);
        assertEquals(131, t.m_points.size());
        TrajectoryPoint p = t.getPoint(40);
        assertEquals(0.5, p.state().state().getPose().getX(), kDelta);
        assertEquals(0, p.state().state().getHeadingRate(), kDelta);
    }

    /**
     * Pure rotation does not work.
     */
    @Test
    void testRotation() {
        List<Pose2d> waypoints = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d(1));
        List<TimingConstraint> constraints = new ArrayList<>();
        Trajectory100 t = TrajectoryPlanner.restToRest(waypoints, headings, constraints);
        assertTrue(t.isEmpty());
    }

}
