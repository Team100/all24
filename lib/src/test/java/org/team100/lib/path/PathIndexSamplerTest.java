package org.team100.lib.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.timing.TimingUtil.TimingException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

class PathIndexSamplerTest {
    public static final double kDelta = 0.001;

    @Test
    void test() throws TimingException {

        List<Pose2dWithMotion> waypoints = Arrays.asList(
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(0.0, 0.0),
                                GeometryUtil.fromDegrees(0)),
                        new Twist2d(1, 0, 0), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(24.0, 0.0),
                                GeometryUtil.fromDegrees(30)),
                        new Twist2d(1, 0, 0), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(36.0, 0.0),
                                GeometryUtil.fromDegrees(60)),
                        new Twist2d(0, 1, 0), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(36.0, 24.0),
                                GeometryUtil.fromDegrees(90)),
                        new Twist2d(1, 0, 0), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(60.0, 24.0),
                                GeometryUtil.fromDegrees(180)),
                        new Twist2d(1, 0, 0), 0, 0));

        // Create the reference trajectory (straight line motion between waypoints).
        Path100 trajectory = new Path100(waypoints);
        PathIndexSampler sampler = new PathIndexSampler(trajectory);

        assertEquals(0.0, sampler.getMinIndex(), kDelta);
        assertEquals(4, sampler.getMaxIndex(), kDelta);

        // initial sample is exactly at the start
        Pose2dWithMotion sample0 = sampler.sample(0.0).state();
        assertEquals(0, sample0.getPose().getX(), kDelta);
        assertEquals(0, sample0.getPose().getY(), kDelta);

        // course is +x
        assertEquals(0, sample0.getCourse().get().getDegrees());

        // heading is 0
        assertEquals(0, sample0.getHeading().getDegrees());

        // these are constant-twist paths, so they are little arcs.
        // since the "index" iterator picks the actual midpoint, the x value
        // is correct but the y value indicates the "sag" between the end points.
        Pose2dWithMotion sample1 = sampler.sample(0.5).state();
        assertEquals(12, sample1.getPose().getX(), kDelta);
        assertEquals(-1.58, sample1.getPose().getY(), kDelta);

        // course should be +x
        assertEquals(0, sample1.getCourse().get().getDegrees(), kDelta);

        // heading should be about 15 degrees (halfway to the 30-degree waypoint)
        assertEquals(15, sample1.getHeading().getDegrees(), kDelta);

        // halfway between the last two points, the path sags a little,
        Pose2dWithMotion sample2 = sampler.sample(3.5).state();
        assertEquals(48, sample2.getPose().getX(), kDelta);
        assertEquals(19.029, sample2.getPose().getY(), kDelta);

        // course should be +x
        assertEquals(0, sample2.getCourse().get().getDegrees(), kDelta);

        // heading should be about 135 degrees, halfway to the goal
        assertEquals(135, sample2.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion sample5 = sampler.sample(2.5).state();
        assertEquals(37.580, sample5.getPose().getX(), kDelta);
        assertEquals(12, sample5.getPose().getY(), kDelta);
        // TODO: completely wrong, this is an artifact of the twist
        assertEquals(45, sample5.getCourse().get().getDegrees());
        assertEquals(75, sample5.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion sample6 = sampler.sample(3.0).state();
        assertEquals(36, sample6.getPose().getX(), kDelta);
        assertEquals(24, sample6.getPose().getY(), kDelta);
        assertEquals(0, sample6.getCourse().get().getDegrees());
        assertEquals(90, sample6.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion sample8 = sampler.sample(4.0).state();
        assertEquals(60, sample8.getPose().getX(), kDelta);
        assertEquals(24, sample8.getPose().getY(), kDelta);
        assertEquals(0, sample8.getCourse().get().getDegrees());
        assertEquals(180, sample8.getHeading().getDegrees(), kDelta);
    }

    @Test
    void testRotate() throws TimingException {
        List<Pose2dWithMotion> waypoints = Arrays.asList(
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(0.0, 0.0),
                                GeometryUtil.fromDegrees(0)),
                        new Twist2d(0, 0, 1),
                        0, 0),
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(0.0, 0.0),
                                GeometryUtil.fromDegrees(30)),
                        new Twist2d(0, 0, 0),
                        0, 0));

        // Create the reference trajectory (straight line motion between waypoints).
        Path100 trajectory = new Path100(waypoints);
        PathIndexSampler sampler = new PathIndexSampler(trajectory);

        assertEquals(0.0, sampler.getMinIndex(), kDelta);
        assertEquals(1.0, sampler.getMaxIndex(), kDelta);

        // initial sample is exactly at the start
        Pose2dWithMotion sample0 = sampler.sample(0.0).state();
        assertEquals(0, sample0.getPose().getX(), kDelta);
        assertEquals(0, sample0.getPose().getY(), kDelta);

        // no linear motion, so no course
        assertTrue(sample0.getCourse().isEmpty());

        // heading is 0
        assertEquals(0, sample0.getHeading().getDegrees());

        // still at the origin
        Pose2dWithMotion sample1 = sampler.sample(0.5).state();
        assertEquals(0, sample1.getPose().getX(), kDelta);
        assertEquals(0, sample1.getPose().getY(), kDelta);

        // still no course
        assertTrue(sample1.getCourse().isEmpty());

        // heading should be about 15 degrees (halfway to the 30-degree waypoint)
        assertEquals(15, sample1.getHeading().getDegrees(), kDelta);

        // still at the origin
        Pose2dWithMotion sample2 = sampler.sample(1.0).state();
        assertEquals(0, sample2.getPose().getX(), kDelta);
        assertEquals(0, sample2.getPose().getY(), kDelta);

        // still no course
        assertTrue(sample2.getCourse().isEmpty());

        // heading is at the final value
        assertEquals(30, sample2.getHeading().getDegrees(), kDelta);
    }

    @Test
    void testBothLinearAndRotate() throws TimingException {
        List<Pose2dWithMotion> waypoints = Arrays.asList(
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(0.0, 0.0),
                                GeometryUtil.fromDegrees(0)),
                        new Twist2d(1, 0, 1), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(23.9, 0.0),
                                GeometryUtil.fromDegrees(10)),
                        new Twist2d(1, 0, 1), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(24.0, 0.0),
                                GeometryUtil.fromDegrees(10)),
                        new Twist2d(0, 0, 0), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(24.0, 0.0),
                                GeometryUtil.fromDegrees(10.1)),
                        new Twist2d(0, 0, 1), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(24.0, 0.0),
                                GeometryUtil.fromDegrees(19.9)),
                        new Twist2d(0, 0, 1), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(24.0, 0.0),
                                GeometryUtil.fromDegrees(20)),
                        new Twist2d(0, 0, 0), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(24.0, 0.1),
                                GeometryUtil.fromDegrees(20)),
                        new Twist2d(0, 1, 1), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(24.0, 24.0),
                                GeometryUtil.fromDegrees(30)),
                        new Twist2d(0, 1, 1), 0, 0));

        // Create the reference trajectory (straight line motion between waypoints).
        Path100 trajectory = new Path100(waypoints);
        PathIndexSampler sampler = new PathIndexSampler(trajectory);

        assertEquals(0.0, sampler.getMinIndex(), kDelta);
        assertEquals(7.0, sampler.getMaxIndex(), kDelta);

        Pose2dWithMotion sample0 = sampler.sample(0.0).state();
        assertEquals(0, sample0.getPose().getX(), kDelta);
        assertEquals(0, sample0.getPose().getY(), kDelta);
        assertEquals(0, sample0.getCourse().get().getDegrees());
        assertEquals(0, sample0.getHeading().getDegrees());

        Pose2dWithMotion sample1 = sampler.sample(0.5).state();
        assertEquals(11.950, sample1.getPose().getX(), kDelta);
        assertEquals(-0.521, sample1.getPose().getY(), kDelta);
        assertEquals(0, sample1.getCourse().get().getDegrees());
        assertEquals(5, sample1.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion sample2 = sampler.sample(2.0).state();
        assertEquals(24, sample2.getPose().getX(), kDelta);
        // at the corner there is no course
        assertTrue(sample2.getCourse().isEmpty());
        assertEquals(10, sample2.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion sample3 = sampler.sample(3.5).state();
        assertEquals(24, sample3.getPose().getX(), kDelta);
        assertEquals(0, sample3.getPose().getY(), kDelta);
        // still no course
        assertTrue(sample3.getCourse().isEmpty());
        assertEquals(15, sample3.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion sample4 = sampler.sample(5.0).state();
        assertEquals(24, sample4.getPose().getX(), kDelta);
        assertEquals(0, sample4.getPose().getY(), kDelta);
        // still no course
        assertTrue(sample4.getCourse().isEmpty());
        assertEquals(20, sample4.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion sample5 = sampler.sample(6.5).state();
        assertEquals(24.522, sample5.getPose().getX(), kDelta);
        assertEquals(12.05, sample5.getPose().getY(), kDelta);
        assertEquals(90, sample5.getCourse().get().getDegrees());
        assertEquals(25, sample5.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion sample6 = sampler.sample(7.0).state();
        assertEquals(24, sample6.getPose().getX(), kDelta);
        assertEquals(24, sample6.getPose().getY(), kDelta);
        assertEquals(90, sample6.getCourse().get().getDegrees());
        assertEquals(30, sample6.getHeading().getDegrees(), kDelta);
    }

}
