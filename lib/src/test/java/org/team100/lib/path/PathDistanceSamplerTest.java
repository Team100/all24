package org.team100.lib.path;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.timing.TimingUtil.TimingException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

class PathDistanceSamplerTest {
    public static final double kDelta = 0.001;

    @Test
    void test() throws TimingException {
        List<Rotation2d> headings = Arrays.asList(
                GeometryUtil.fromDegrees(0),
                GeometryUtil.fromDegrees(30),
                GeometryUtil.fromDegrees(60),
                GeometryUtil.fromDegrees(90),
                GeometryUtil.fromDegrees(180));

        List<Pose2dWithMotion> waypoints = Arrays.asList(
                new Pose2dWithMotion(
                        new Pose2d(new Translation2d(0.0, 0.0), headings.get(0)),
                        new Twist2d(1, 0, 0), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(new Translation2d(24.0, 0.0), headings.get(1)),
                        new Twist2d(1, 0, 0), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(new Translation2d(36.0, 0.0), headings.get(2)),
                        new Twist2d(0, 1, 0), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(new Translation2d(36.0, 24.0), headings.get(3)),
                        new Twist2d(1, 0, 0), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(new Translation2d(60.0, 24.0), headings.get(4)),
                        new Twist2d(1, 0, 0), 0, 0));

        // Create the reference trajectory (straight line motion between waypoints).
        Path100 trajectory = new Path100(waypoints);
        PathDistanceSampler sampler = new PathDistanceSampler(trajectory);

        assertEquals(0.0, sampler.getMinDistance(), kDelta);
        // the total path length is a bit more than the straight-line path because each path is a constant-twist arc.
        assertEquals(87.348, sampler.getMaxDistance(), kDelta);

        // initial sample is exactly at the start
        Pose2dWithMotion sample0 = sampler.sample(0.0).state();
        assertEquals(0, sample0.getPose().getX(), kDelta);
        assertEquals(0, sample0.getPose().getY(), kDelta);

        // course is +x
        assertEquals(0, sample0.getCourse().get().getDegrees());
        
        // heading is 0
        assertEquals(0, sample0.getHeading().getDegrees());

        // these are constant-twist paths, so they are little arcs.
        // halfway between 0 and 1, the path sags a little, and it's a little longer,
        // so this is not (12,0).
        Pose2dWithMotion sample12 = sampler.sample(12.0).state();
        assertEquals(11.862, sample12.getPose().getX(), kDelta);
        assertEquals(-1.58, sample12.getPose().getY(), kDelta);

        // course should be +x
        assertEquals(0, sample12.getCourse().get().getDegrees(), kDelta);

        // heading should be about 15 degrees, but since the path is a bit
        // longer than the straight line, we're not quite to the middle of it yet
        assertEquals(14.829, sample12.getHeading().getDegrees(), kDelta);

        // halfway between the last two points, the path sags a little,
        // and it's a little longer, so this is not (48,0)
        Pose2dWithMotion sample72 = sampler.sample(72.0).state();
        assertEquals(45.985, sample72.getPose().getX(), kDelta);
        assertEquals(19.149, sample72.getPose().getY(), kDelta);

        // course should be +x
        assertEquals(0, sample72.getCourse().get().getDegrees(), kDelta);

        // heading should be about 135 degrees, but because of the longer
        // paths, we're not quite to the center of the arc yet
        assertEquals(128.181, sample72.getHeading().getDegrees(), kDelta);
    }

}
