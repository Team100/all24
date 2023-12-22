package org.team100.lib.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class Path100Test {
    private static final double kDelta = 0.001;

    private static final List<Rotation2d> kHeadings = Arrays.asList(
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(30),
            GeometryUtil.fromDegrees(60),
            GeometryUtil.fromDegrees(90));

    private static final List<Pose2dWithMotion> kWaypoints = Arrays.asList(
            new Pose2dWithMotion(new Pose2d(new Translation2d(0.0, 0.0), kHeadings.get(0)), 0),
            new Pose2dWithMotion(new Pose2d(new Translation2d(24.0, 0.0), kHeadings.get(1)), 0),
            new Pose2dWithMotion(new Pose2d(new Translation2d(36.0, 12.0), kHeadings.get(2)), 0),
            new Pose2dWithMotion(new Pose2d(new Translation2d(60.0, 12.0), kHeadings.get(3)), 0));

    @Test
    void testConstruction() {
        Path100 traj = new Path100(kWaypoints);
        assertFalse(traj.isEmpty());
        assertEquals(4, traj.length());
    }

    @Test
    void testStateAccessors() {
        Path100 traj = new Path100(kWaypoints);

        assertEquals(kWaypoints.get(0), traj.getPoint(0).state());
        assertEquals(kWaypoints.get(1), traj.getPoint(1).state());
        assertEquals(kWaypoints.get(2), traj.getPoint(2).state());
        assertEquals(kWaypoints.get(3), traj.getPoint(3).state());

        assertEquals(kHeadings.get(0), traj.getPoint(0).state().getHeading());
        assertEquals(kHeadings.get(1), traj.getPoint(1).state().getHeading());
        assertEquals(kHeadings.get(2), traj.getPoint(2).state().getHeading());
        assertEquals(kHeadings.get(3), traj.getPoint(3).state().getHeading());
    }

    @Test
    void testInterpolateExact() {
        Path100 traj = new Path100(kWaypoints);

        PathSamplePoint interpolated0 = traj.getInterpolated(0.0);
        assertEquals(0, interpolated0.state().getPose().getX(), kDelta);
        assertEquals(0, interpolated0.state().getPose().getY(), kDelta);
        assertEquals(0, interpolated0.state().getHeading().getDegrees(), kDelta);
        assertEquals(0, interpolated0.getIndexFloor());
        assertEquals(0, interpolated0.getIndexCeil());

        PathSamplePoint interpolated1 = traj.getInterpolated(1.0);
        assertEquals(24, interpolated1.state().getPose().getX(), kDelta);
        assertEquals(0, interpolated1.state().getPose().getY(), kDelta);
        assertEquals(30, interpolated1.state().getHeading().getDegrees(), kDelta);
        assertEquals(1, interpolated1.getIndexFloor());
        assertEquals(1, interpolated1.getIndexCeil());

        PathSamplePoint interpolated2 = traj.getInterpolated(2.0);
        assertEquals(36, interpolated2.state().getPose().getX(), kDelta);
        assertEquals(12, interpolated2.state().getPose().getY(), kDelta);
        assertEquals(60, interpolated2.state().getHeading().getDegrees(), kDelta);
        assertEquals(2, interpolated2.getIndexFloor());
        assertEquals(2, interpolated2.getIndexCeil());

        PathSamplePoint interpolated3 = traj.getInterpolated(3.0);
        assertEquals(60, interpolated3.state().getPose().getX(), kDelta);
        assertEquals(12, interpolated3.state().getPose().getY(), kDelta);
        assertEquals(90, interpolated3.state().getHeading().getDegrees(), kDelta);
        assertEquals(3, interpolated3.getIndexFloor());
        assertEquals(3, interpolated3.getIndexCeil());
    }

    /** The constant-twist arcs between states mean these samples don't quite match the straight-line paths. */
    @Test
    void testInterpolateBetween() {
        Path100 traj = new Path100(kWaypoints);

        PathSamplePoint interpolated025 = traj.getInterpolated(0.25);
        assertEquals(5.948, interpolated025.state().getTranslation().getX(), kDelta);
        assertEquals(-1.183, interpolated025.state().getTranslation().getY(), kDelta);
        assertEquals(7.5, interpolated025.state().getHeading().getDegrees(), kDelta);
        assertEquals(0, interpolated025.getIndexFloor());
        assertEquals(1, interpolated025.getIndexCeil());

        PathSamplePoint interpolated150 = traj.getInterpolated(1.5);
        assertEquals(30.789, interpolated150.state().getTranslation().getX(), kDelta);
        assertEquals(5.210, interpolated150.state().getTranslation().getY(), kDelta);
        assertEquals(45, interpolated150.state().getHeading().getDegrees(), kDelta);
        assertEquals(1, interpolated150.getIndexFloor());
        assertEquals(2, interpolated150.getIndexCeil());

        PathSamplePoint interpolated275 = traj.getInterpolated(2.75);
        assertEquals(54.051, interpolated275.state().getTranslation().getX(), kDelta);
        assertEquals(10.816, interpolated275.state().getTranslation().getY(), kDelta);
        assertEquals(82.5, interpolated275.state().getHeading().getDegrees(), kDelta);
        assertEquals(2, interpolated275.getIndexFloor());
        assertEquals(3, interpolated275.getIndexCeil());
    }
}
