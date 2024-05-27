package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.path.Path100;
import org.team100.lib.path.PathPoint;
import org.team100.lib.path.PathSamplePoint;
import org.team100.lib.spline.HolonomicSpline;
import org.team100.lib.spline.SplineGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class TrajectoryUtilTest {
    private static final double kDelta = 0.01;

    @Test
    void testEmpty() {
        List<HolonomicSpline> splines = new ArrayList<>();
        double maxDx = 0.1;
        double maxDy = 0.1;
        double maxDTheta = 0.1;
        Path100 path = new Path100(SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
        assertEquals(0, path.length(), 0.001);
    }

    @Test
    void testSimple() {
        // spline is in the x direction, no curvature.
        HolonomicSpline spline = new HolonomicSpline(
                new Pose2d(),
                new Pose2d(1, 0, new Rotation2d()),
                new Rotation2d(), new Rotation2d()) {

            @Override
            public Translation2d getPoint(double t) {
                return new Translation2d(t, 0);
            }

            @Override
            public Rotation2d getHeading(double t) {
                return GeometryUtil.kRotationZero;
            }

            @Override
            public Optional<Rotation2d> getCourse(double t) {
                return Optional.of(GeometryUtil.kRotationZero);
            }

            @Override
            public double getDHeading(double t) {
                return 0;
            }

            @Override
            public double getCurvature(double t) {
                return 0;
            }

            @Override
            public double getDCurvature(double t) {
                return 0;
            }

            @Override
            public double getVelocity(double t) {
                return 1;
            }
        };
        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(spline);
        double maxDx = 0.1;
        double maxDy = 0.1;
        double maxDTheta = 0.1;
        Path100 path = new Path100(SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
        assertEquals(2, path.length(), 0.001);
        {
            PathSamplePoint sample = path.getInterpolated(0);
            Pose2dWithMotion pose = sample.state();
            Pose2d pose2d = pose.getPose();
            assertEquals(0, pose2d.getX(), 0.001);
            assertEquals(0, pose2d.getY(), 0.001);
            assertEquals(0, pose2d.getRotation().getRadians(), 0.001);
        }
        {
            PathSamplePoint sample = path.getInterpolated(0.5);
            Pose2dWithMotion pose = sample.state();
            Pose2d pose2d = pose.getPose();
            assertEquals(0.5, pose2d.getX(), 0.001);
            assertEquals(0, pose2d.getY(), 0.001);
            assertEquals(0, pose2d.getRotation().getRadians(), 0.001);
        }
        {
            PathSamplePoint sample = path.getInterpolated(1);
            Pose2dWithMotion pose = sample.state();
            Pose2d pose2d = pose.getPose();
            assertEquals(1.0, pose2d.getX(), 0.001);
            assertEquals(0, pose2d.getY(), 0.001);
            assertEquals(0, pose2d.getRotation().getRadians(), 0.001);
        }
    }

    /**
     * Stationary paths don't work.
     */
    @Test
    void testStationary() {
        List<Pose2d> waypoints = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d());
        assertThrows(IllegalArgumentException.class,
                () -> TrajectoryUtil100.trajectoryFromWaypointsAndHeadings(
                        waypoints, headings, 0.01, 0.01, 0.1));

    }

    @Test
    void testLinear() {
        List<Pose2d> waypoints = List.of(new Pose2d(), new Pose2d(1, 0, new Rotation2d()));
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d());
        Path100 path = TrajectoryUtil100.trajectoryFromWaypointsAndHeadings(
                waypoints, headings, 0.01, 0.01, 0.1);

        assertEquals(2, path.length());
        PathPoint p = path.getPoint(0);
        assertEquals(0, p.state().getPose().getX(), kDelta);
        assertEquals(0, p.state().getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.state().getHeadingRate(), kDelta);
        p = path.getPoint(1);
        assertEquals(1, p.state().getPose().getX(), kDelta);
        assertEquals(0, p.state().getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.state().getHeadingRate(), kDelta);
    }

    /**
     * Stationary pure-rotation paths don't work.
     */
    @Test
    void testRotation() {
        List<Pose2d> waypoints = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d(1));
        assertThrows(IllegalArgumentException.class,
                () -> TrajectoryUtil100.trajectoryFromWaypointsAndHeadings(
                        waypoints, headings, 0.01, 0.01, 0.1));
    }

    /** Preserves the tangent at the corner and so makes a little "S" */
    @Test
    void testCorner() {
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d()),
                new Pose2d(1, 1, new Rotation2d(Math.PI / 2)));
        List<Rotation2d> headings = List.of(
                new Rotation2d(),
                new Rotation2d(),
                new Rotation2d());
        Path100 path = TrajectoryUtil100.trajectoryFromWaypointsAndHeadings(
                waypoints, headings, 0.01, 0.01, 0.1);

        assertEquals(9, path.length());
        PathPoint p = path.getPoint(0);
        assertEquals(0, p.state().getPose().getX(), kDelta);
        assertEquals(0, p.state().getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.state().getHeadingRate(), kDelta);
        p = path.getPoint(1);
        assertEquals(1, p.state().getPose().getX(), kDelta);
        assertEquals(0, p.state().getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.state().getHeadingRate(), kDelta);
    }

    /**
     * Paths with corners don't work.
     */
    @Test
    void testActualCorner() {
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d(Math.PI / 2)),
                new Pose2d(1, 1, new Rotation2d(Math.PI / 2)));
        List<Rotation2d> headings = List.of(
                new Rotation2d(),
                new Rotation2d(),
                new Rotation2d(),
                new Rotation2d());
        assertThrows(IllegalArgumentException.class,
                () -> TrajectoryUtil100.trajectoryFromWaypointsAndHeadings(
                        waypoints, headings, 0.01, 0.01, 0.1));
    }

    /**
     * Partially-stationary paths don't work.
     */
    @Test
    void testComposite() {
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d()),
                new Pose2d(2, 0, new Rotation2d()));
        List<Rotation2d> headings = List.of(
                new Rotation2d(),
                new Rotation2d(),
                new Rotation2d(1),
                new Rotation2d(1));
        assertThrows(IllegalArgumentException.class,
                () -> TrajectoryUtil100.trajectoryFromWaypointsAndHeadings(
                        waypoints, headings, 0.01, 0.01, 0.1));
    }
}
