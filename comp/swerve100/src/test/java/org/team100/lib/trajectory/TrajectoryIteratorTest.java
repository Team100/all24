package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.path.Path;
import org.team100.lib.path.PathIndexIterator;
import org.team100.lib.path.PathIndexSampler;
import org.team100.lib.path.PathSamplePoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class TrajectoryIteratorTest {
    public static final double kTestEpsilon = 1e-12;

    public static final List<Pose2dWithMotion> kWaypoints = Arrays.asList(
            new Pose2dWithMotion(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()), 0),
            new Pose2dWithMotion(new Pose2d(new Translation2d(24.0, 0.0), new Rotation2d()), 0),
            new Pose2dWithMotion(new Pose2d(new Translation2d(36.0, 12.0), new Rotation2d()), 0),
            new Pose2dWithMotion(new Pose2d(new Translation2d(60.0, 12.0), new Rotation2d()), 0));

    List<Rotation2d> kHeadings = Arrays.asList(
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(30),
            GeometryUtil.fromDegrees(60),
            GeometryUtil.fromDegrees(90),
            GeometryUtil.fromDegrees(180));

    private static void assertPoseWithCurvatureEquals(Pose2dWithMotion a, Pose2dWithMotion b) {
        assertEquals(a,b);
    }
    
    @Test
    void test() {
        Path traj = new Path(kWaypoints);
        PathIndexIterator iterator = new PathIndexIterator(new PathIndexSampler(traj));

        // Initial conditions.
        assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertEquals(kWaypoints.get(0), iterator.getState());
        assertFalse(iterator.isDone());

        // Advance forward.
        assertPoseWithCurvatureEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.5),
                iterator.preview(0.5).state());
        // assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), 0.5),
        //         iterator.preview(0.5).heading());
        PathSamplePoint newPoint = iterator.advance(0.5);

        assertPoseWithCurvatureEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.5), newPoint.state());
        // assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), 0.5), newPoint.heading());
        assertEquals(0.5, iterator.getProgress(), kTestEpsilon);
        assertEquals(2.5, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());

        // Advance backwards.
        assertPoseWithCurvatureEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.25),
                iterator.preview(-0.25).state());
        // assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), 0.25),
        //         iterator.preview(-0.25).heading());
        newPoint = iterator.advance(-0.25);
        
        assertPoseWithCurvatureEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.25), newPoint.state());
        // assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), 0.25), newPoint.heading());
        assertEquals(0.25, iterator.getProgress(), kTestEpsilon);
        assertEquals(2.75, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());

        // Advance past end.
        assertEquals(kWaypoints.get(3), iterator.preview(5.0).state());
        // assertEquals(kHeadings.get(3), iterator.preview(5.0).heading());
        newPoint = iterator.advance(5.0);
        assertEquals(kWaypoints.get(3), newPoint.state());
        // assertEquals(kHeadings.get(3), newPoint.heading());
        assertEquals(3.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(0.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertTrue(iterator.isDone());

        // Advance past beginning.
        assertEquals(kWaypoints.get(0), iterator.preview(-5.0).state());
        // assertEquals(kHeadings.get(0), iterator.preview(-5.0).heading());
        newPoint = iterator.advance(-5.0);
        assertEquals(kWaypoints.get(0), newPoint.state());
        // assertEquals(kHeadings.get(0), newPoint.heading());
        assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());
    }

}
