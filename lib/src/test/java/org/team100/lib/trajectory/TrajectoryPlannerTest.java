package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.timing.TimingConstraint;

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
        double start_vel = 0;
        double end_vel = 0;
        double max_vel = 1;
        double max_accel = 1;

        Trajectory100 t = TrajectoryPlanner.generateTrajectory(
                waypoints, headings, constraints, start_vel, end_vel,
                max_vel, max_accel);
        assertTrue(t.isEmpty());
    }

    @Test
    void testLinear() {
        List<Pose2d> waypoints = List.of(new Pose2d(), new Pose2d(1, 0, new Rotation2d()));
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d());
        List<TimingConstraint> constraints = new ArrayList<>();
        double start_vel = 0;
        double end_vel = 0;
        double max_vel = 1;
        double max_accel = 1;
        Trajectory100 t = TrajectoryPlanner.generateTrajectory(waypoints, headings, constraints, start_vel, end_vel,
                max_vel, max_accel);
        assertEquals(80, t.m_points.size());
        TrajectoryPoint p = t.getPoint(40);
        assertEquals(0.5, p.state().state().getPose().getX(), kDelta);
        assertEquals(0, p.state().state().getHeadingRate(), kDelta);
    }

    /**
     * Is trajectory planning fast enough to run every loop?
     * This computes a single spline, and makes a schedule along it.
     */
    @Test
    void testPerformance() {
        List<Pose2d> waypoints = List.of(new Pose2d(), new Pose2d(1, 1, new Rotation2d(Math.PI/2)));
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d());
        List<TimingConstraint> constraints = new ArrayList<>();
        double start_vel = 0;
        double end_vel = 0;
        double max_vel = 2;
        double max_accel = 2;
        long startTimeNs = System.nanoTime();
        Trajectory100 t = new Trajectory100();
        final int iterations = 100;
        for (int i = 0; i < iterations; ++i) {
            t = TrajectoryPlanner.generateTrajectory(waypoints, headings, constraints, start_vel, end_vel,
                    max_vel, max_accel);
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
        double start_vel = 0;
        double end_vel = 0;
        double max_vel = 1;
        double max_accel = 1;
        Trajectory100 t = TrajectoryPlanner.generateTrajectory(
                waypoints, headings, constraints, start_vel, end_vel,
                max_vel, max_accel);
        assertTrue(t.isEmpty());
    }

}
