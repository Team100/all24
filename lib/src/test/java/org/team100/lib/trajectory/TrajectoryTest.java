package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.path.Path;
import org.team100.lib.path.PathSamplePoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class TrajectoryTest {
    private static final List<Pose2dWithMotion> kWaypoints = Arrays.asList(
            new Pose2dWithMotion(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()), 0),
            new Pose2dWithMotion(new Pose2d(new Translation2d(24.0, 0.0), new Rotation2d()), 0),
            new Pose2dWithMotion(new Pose2d(new Translation2d(36.0, 12.0), new Rotation2d()), 0),
            new Pose2dWithMotion(new Pose2d(new Translation2d(60.0, 12.0), new Rotation2d()), 0));

    // private static final List<Rotation2d> kHeadings = Arrays.asList(
    //         GeometryUtil.fromDegrees(0),
    //         GeometryUtil.fromDegrees(30),
    //         GeometryUtil.fromDegrees(60),
    //         GeometryUtil.fromDegrees(90),
    //         GeometryUtil.fromDegrees(180));

    @Test
    void testConstruction() {
        Path traj = new Path(kWaypoints);
        assertFalse(traj.isEmpty());
        assertEquals(4, traj.length());
    }

    @Test
    void testStateAccessors() {
        Path traj = new Path(kWaypoints);

        assertEquals(kWaypoints.get(0), traj.getPoint(0).state());
        assertEquals(kWaypoints.get(1), traj.getPoint(1).state());
        assertEquals(kWaypoints.get(2), traj.getPoint(2).state());
        assertEquals(kWaypoints.get(3), traj.getPoint(3).state());

        // assertEquals(kHeadings.get(0), traj.getPoint(0).heading());
        // assertEquals(kHeadings.get(1), traj.getPoint(1).heading());
        // assertEquals(kHeadings.get(2), traj.getPoint(2).heading());
        // assertEquals(kHeadings.get(3), traj.getPoint(3).heading());

        assertEquals(kWaypoints.get(0), traj.getInterpolated(0.0).state());
        assertEquals(0, traj.getInterpolated(0.0).index_floor());
        assertEquals(0, traj.getInterpolated(0.0).index_ceil());
        assertEquals(kWaypoints.get(1), traj.getInterpolated(1.0).state());
        assertEquals(1, traj.getInterpolated(1.0).index_floor());
        assertEquals(1, traj.getInterpolated(1.0).index_ceil());
        assertEquals(kWaypoints.get(2), traj.getInterpolated(2.0).state());
        assertEquals(2, traj.getInterpolated(2.0).index_floor());
        assertEquals(2, traj.getInterpolated(2.0).index_ceil());
        assertEquals(kWaypoints.get(3), traj.getInterpolated(3.0).state());
        assertEquals(3, traj.getInterpolated(3.0).index_floor());
        assertEquals(3, traj.getInterpolated(3.0).index_ceil());

        // assertEquals(kHeadings.get(0), traj.getInterpolated(0.0).heading());
        assertEquals(0, traj.getInterpolated(0.0).index_floor());
        assertEquals(0, traj.getInterpolated(0.0).index_ceil());
        // assertEquals(kHeadings.get(1), traj.getInterpolated(1.0).heading());
        assertEquals(1, traj.getInterpolated(1.0).index_floor());
        assertEquals(1, traj.getInterpolated(1.0).index_ceil());
        // assertEquals(kHeadings.get(2), traj.getInterpolated(2.0).heading());
        assertEquals(2, traj.getInterpolated(2.0).index_floor());
        assertEquals(2, traj.getInterpolated(2.0).index_ceil());
        // assertEquals(kHeadings.get(3), traj.getInterpolated(3.0).heading());
        assertEquals(3, traj.getInterpolated(3.0).index_floor());
        assertEquals(3, traj.getInterpolated(3.0).index_ceil());

        assertEquals(
                kWaypoints.get(0).getPose().getTranslation().interpolate(
                        kWaypoints.get(1).getPose().getTranslation(),
                        .25),
                traj.getInterpolated(0.25).state().getPose().getTranslation());
        assertEquals(0, traj.getInterpolated(0.25).index_floor());
        assertEquals(1, traj.getInterpolated(0.25).index_ceil());
        assertEquals(
                kWaypoints.get(1).getPose().getTranslation().interpolate(
                        kWaypoints.get(2).getPose().getTranslation(),
                        .5),
                traj.getInterpolated(1.5).state().getPose().getTranslation());
        assertEquals(1, traj.getInterpolated(1.5).index_floor());
        assertEquals(2, traj.getInterpolated(1.5).index_ceil());
        assertEquals(
                kWaypoints.get(2).getPose().getTranslation().interpolate(
                        kWaypoints.get(3).getPose().getTranslation(),
                        .75),
                traj.getInterpolated(2.75).state().getPose().getTranslation());
        assertEquals(2, traj.getInterpolated(2.75).index_floor());
        assertEquals(3, traj.getInterpolated(2.75).index_ceil());

        // assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), .25),
        //         traj.getInterpolated(0.25).heading());
        assertEquals(0, traj.getInterpolated(0.25).index_floor());
        assertEquals(1, traj.getInterpolated(0.25).index_ceil());
        // assertEquals(kHeadings.get(1).interpolate(kHeadings.get(2), .5),
        //         traj.getInterpolated(1.5).heading());
        assertEquals(1, traj.getInterpolated(1.5).index_floor());
        assertEquals(2, traj.getInterpolated(1.5).index_ceil());
        // assertEquals(kHeadings.get(2).interpolate(kHeadings.get(3), .75),
        //         traj.getInterpolated(2.75).heading());
        assertEquals(2, traj.getInterpolated(2.75).index_floor());
        assertEquals(3, traj.getInterpolated(2.75).index_ceil());

        PathSamplePoint sample0 = traj.getInterpolated(0.25);
        assertEquals(
                kWaypoints.get(0).getPose().getTranslation().interpolate(
                        kWaypoints.get(1).getPose().getTranslation(),
                        .25),
                sample0.state().getPose().getTranslation());

        PathSamplePoint sample1 = traj.getInterpolated(1.5);
        assertEquals(
                kWaypoints.get(1).getPose().getTranslation().interpolate(
                        kWaypoints.get(2).getPose().getTranslation(),
                        .5),
                sample1.state().getPose().getTranslation());

        PathSamplePoint sample2 = traj.getInterpolated(2.75);
        assertEquals(
                kWaypoints.get(2).getPose().getTranslation().interpolate(
                        kWaypoints.get(3).getPose().getTranslation(),
                        .75),
                sample2.state().getPose().getTranslation());

        // PathSamplePoint sample3 = traj.getInterpolated(0.25);
        // assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), .25),
        //         sample3.heading());

        // PathSamplePoint sample4 = traj.getInterpolated(1.5);
        // assertEquals(kHeadings.get(1).interpolate(kHeadings.get(2), .5),
        //         sample4.heading());

        // PathSamplePoint sample5 = traj.getInterpolated(2.75);
        // assertEquals(kHeadings.get(2).interpolate(kHeadings.get(3), .75),
        //         sample5.heading());
    }
}
