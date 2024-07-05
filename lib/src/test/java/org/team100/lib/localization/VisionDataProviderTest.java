package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.telemetry.TestLogger;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

class VisionDataProviderTest implements Timeless {
    private static final double kDelta = 0.01;
    private static final Logger logger = new TestLogger();

    FireControl f = new FireControl() {
    };

    @Test
    void testEstimateRobotPose() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        // these lists receive the updates
        final List<Pose2d> poseEstimate = new ArrayList<Pose2d>();
        final List<Double> timeEstimate = new ArrayList<Double>();

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void addVisionMeasurement(Pose2d p, double t, double[] sd1, double[] sd2) {
                poseEstimate.add(p);
                timeEstimate.add(t);
            }

            @Override
            public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
                return Optional.of(GeometryUtil.kRotationZero);
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(logger, layout, poseEstimator, f);

        // in red layout blip 7 is on the other side of the field

        // one meter range (Z forward)
        Blip24 blip = new Blip24(7, new Transform3d(new Translation3d(0, 0, 1), new Rotation3d()));

        // verify tag location
        Pose3d tagPose = layout.getTagPose(Alliance.Red, 7).get();
        assertEquals(16.5791, tagPose.getX(), kDelta);
        assertEquals(2.663, tagPose.getY(), kDelta);
        assertEquals(1.451, tagPose.getZ(), kDelta);
        assertEquals(0, tagPose.getRotation().getX(), kDelta);
        assertEquals(0, tagPose.getRotation().getY(), kDelta);
        assertEquals(0, tagPose.getRotation().getZ(), kDelta);

        final String key = "foo";
        final Blip24[] blips = new Blip24[] {
                blip
        };

        vdp.estimateRobotPose(key, blips, Alliance.Red);
        // do it twice to convince vdp it's a good estimate
        vdp.estimateRobotPose(key, blips, Alliance.Red);
        assertEquals(1, poseEstimate.size());
        assertEquals(1, timeEstimate.size());

        Pose2d result = poseEstimate.get(0);
        assertEquals(15.5791, result.getX(), kDelta); // target is one meter in front
        assertEquals(2.663, result.getY(), kDelta); // same y as target
        assertEquals(0, result.getRotation().getRadians(), kDelta); // facing along x
    }

    @Test
    void testEstimateRobotPose2() throws IOException {
        // robot is panned right 45, translation is ignored.
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        final List<Pose2d> poseEstimate = new ArrayList<Pose2d>();
        final List<Double> timeEstimate = new ArrayList<Double>();
        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void addVisionMeasurement(Pose2d p, double t, double[] sd1, double[] sd2) {
                poseEstimate.add(p);
                timeEstimate.add(t);
            }

            @Override
            public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
                return Optional.of(new Rotation2d(-Math.PI / 4));
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(logger, layout, poseEstimator, f);

        // camera sees the tag straight ahead in the center of the frame,
        // but rotated pi/4 to the left. this is ignored anyway.
        Blip24 blip = new Blip24(7, new Transform3d(
                new Translation3d(0, 0, Math.sqrt(2)),
                new Rotation3d(0, -Math.PI / 4, 0)));

        // verify tag 7 location
        Pose3d tagPose = layout.getTagPose(Alliance.Red, 7).get();
        assertEquals(16.5791, tagPose.getX(), kDelta);
        assertEquals(2.663, tagPose.getY(), kDelta);
        assertEquals(1.451, tagPose.getZ(), kDelta);
        assertEquals(0, tagPose.getRotation().getX(), kDelta);
        assertEquals(0, tagPose.getRotation().getY(), kDelta);
        assertEquals(0, tagPose.getRotation().getZ(), kDelta);

        // default camera offset is no offset.
        final String cameraSerialNumber = "foo";
        final Blip24[] blips = new Blip24[] { blip };

        vdp.estimateRobotPose(cameraSerialNumber, blips, Alliance.Red);

        // two good estimates are required, so do another one.
        vdp.estimateRobotPose(cameraSerialNumber, blips, Alliance.Red);

        assertEquals(1, poseEstimate.size());
        assertEquals(1, timeEstimate.size());

        Pose2d result = poseEstimate.get(0);
        // robot is is one meter away from the target in x
        assertEquals(15.5791, result.getX(), kDelta);
        // robot is one meter to the left (i.e. in y)
        assertEquals(3.663, result.getY(), kDelta);
        // facing diagonal, this is just what we provided.
        assertEquals(-Math.PI / 4, result.getRotation().getRadians(), kDelta);

        // the delay is just what we told it to use.
        double now = Timer.getFPGATimestamp();
        Double t = timeEstimate.get(0);
        double delay = now - t;
        assertEquals(0.075, delay, kDelta);
    }

    @Test
    void testRotationInterpolation() {
        // just to be sure of what it's doing
        Rotation2d a = Rotation2d.fromDegrees(10);
        Rotation2d b = Rotation2d.fromDegrees(340);
        Rotation2d c = a.interpolate(b, 0.5);
        assertEquals(-5, c.getDegrees(), kDelta);
    }

    @Test
    void testCase1() throws IOException {

        // the case from 2/14
        // robot 45 degrees to the right (negative), so 135 degrees
        // x = 2.2m, y = - 1.3 m from the center speaker tag
        // camera B
        // camera to tag 4: z=2.4, x=0, y=0 (approx)
        // camera to tag 3: z=2.8, x=0.1, y=0.1 (approx)
        // tag 4 in red is at about (0, 2.5)
        // tag 3 in red is at about (0, 3)

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void addVisionMeasurement(Pose2d p, double t, double[] sd1, double[] sd2) {
                //
            }

            @Override
            public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
                return Optional.of(new Rotation2d(3 * Math.PI / 4));
            }
        };
        VisionDataProvider24 vdp = new VisionDataProvider24(logger, layout, poseEstimator, f);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 2.4),
                new Rotation3d()));
        Blip24 tag3 = new Blip24(3, new Transform3d(
                new Translation3d(0.1, 0.1, 2.8),
                new Rotation3d()));

        final String cameraSerialNumber = "1000000013c9c96c";
        final Blip24[] tags = new Blip24[] { tag3, tag4 };

        Experiments.instance.testOverride(Experiment.Triangulate, false);

        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
    }

    @Test
    void testCase2() throws IOException {

        // 1m in front of tag 4
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void addVisionMeasurement(Pose2d p, double t, double[] sd1, double[] sd2) {
                assertEquals(0.96, p.getX(), kDelta);
                assertEquals(2.66, p.getY(), kDelta);
            }

            @Override
            public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
                return Optional.of(new Rotation2d(Math.PI));
            }
        };
        VisionDataProvider24 vdp = new VisionDataProvider24(logger, layout, poseEstimator, f);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));

        // test camera has zero offset
        final String cameraSerialNumber = "test";
        final Blip24[] tags = new Blip24[] { tag4 };

        Experiments.instance.testOverride(Experiment.Triangulate, false);

        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
    }

    @Test
    void testCase2WithOffset() throws IOException {
        // 1m in front of tag 4
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void addVisionMeasurement(Pose2d p, double t, double[] sd1, double[] sd2) {
                assertEquals(1.96, p.getX(), kDelta);
                assertEquals(2.66, p.getY(), kDelta);
            }

            @Override
            public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
                return Optional.of(new Rotation2d(Math.PI));
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(logger, layout, poseEstimator, f);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));

        // test2 camera is 1m in front, so robot is 1m further away.
        final String cameraSerialNumber = "test2";
        final Blip24[] tags = new Blip24[] { tag4 };

        Experiments.instance.testOverride(Experiment.Triangulate, false);

        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
    }

    @Test
    void testCase2WithTriangulation() throws IOException {

        // 1m in front of tag 4
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void addVisionMeasurement(Pose2d p, double t, double[] sd1, double[] sd2) {
                assertEquals(0.96, p.getX(), kDelta);
                assertEquals(2.66, p.getY(), kDelta);
            }

            @Override
            public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
                return Optional.of(new Rotation2d(Math.PI));
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(logger, layout, poseEstimator, f);

        Blip24 tag3 = new Blip24(3, new Transform3d(
                new Translation3d(0.561, 0, 1),
                new Rotation3d()));
        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));

        // test camera has zero offset
        final String cameraSerialNumber = "test";
        final Blip24[] tags = new Blip24[] { tag3, tag4 };

        Experiments.instance.testOverride(Experiment.Triangulate, true);

        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
    }

    @Test
    void testCase2tilt() throws IOException {

        // 1m in front of tag 4, tilted up 45
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void addVisionMeasurement(Pose2d p, double t, double[] sd1, double[] sd2) {
                assertEquals(0.96, p.getX(), kDelta);
                assertEquals(2.66, p.getY(), kDelta);
            }

            @Override
            public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
                return Optional.of(new Rotation2d(Math.PI));
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(logger, layout, poseEstimator, f);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));

        // test camera has zero offset
        final String cameraSerialNumber = "test1";
        final Blip24[] tags = new Blip24[] { tag4 };

        Experiments.instance.testOverride(Experiment.Triangulate, false);

        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
    }

    @Test
    void testCase3() throws IOException {

        // 1m in front of tag 4, 1m to the right
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void addVisionMeasurement(Pose2d p, double t, double[] sd1, double[] sd2) {
                assertEquals(0.96, p.getX(), kDelta);
                assertEquals(3.66, p.getY(), kDelta);
            }

            @Override
            public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
                return Optional.of(new Rotation2d(Math.PI));
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(logger, layout, poseEstimator, f);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(-1, 0, 1),
                new Rotation3d()));

        // test camera has zero offset
        final String cameraSerialNumber = "test";
        final Blip24[] tags = new Blip24[] { tag4 };

        Experiments.instance.testOverride(Experiment.Triangulate, false);

        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
    }

    @Test
    void testCase4() throws IOException {

        // 1m in front of tag 4, 1m to the right, rotated to the left
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void addVisionMeasurement(Pose2d p, double t, double[] sd1, double[] sd2) {
                assertEquals(0.96, p.getX(), kDelta);
                assertEquals(3.66, p.getY(), kDelta);
            }

            @Override
            public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
                return Optional.of(new Rotation2d(-3 * Math.PI / 4));
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(logger, layout, poseEstimator, f);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));

        // test camera has zero offset
        final String cameraSerialNumber = "test";
        final Blip24[] tags = new Blip24[] { tag4 };

        Experiments.instance.testOverride(Experiment.Triangulate, false);

        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
    }

    @Test
    void testCase5() throws IOException {

        // 1m in front of tag 4, 1m to the left, rotated to the right
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void addVisionMeasurement(Pose2d p, double t, double[] sd1, double[] sd2) {
                assertEquals(0.96, p.getX(), kDelta);
                assertEquals(1.66, p.getY(), kDelta);
            }

            @Override
            public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
                return Optional.of(new Rotation2d(3 * Math.PI / 4));
            }
        };
        VisionDataProvider24 vdp = new VisionDataProvider24(logger, layout, poseEstimator, f);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));

        // test camera has zero offset
        final String cameraSerialNumber = "test";
        final Blip24[] tags = new Blip24[] { tag4 };

        Experiments.instance.testOverride(Experiment.Triangulate, false);

        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
    }

    @Test
    void testCase6() throws IOException {

        // 1m in front of tag 4, 1m to the left, rotated to the right
        // looking up at a 45 degree angle
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void addVisionMeasurement(Pose2d p, double t, double[] sd1, double[] sd2) {
                assertEquals(0.96, p.getX(), kDelta);
                assertEquals(1.66, p.getY(), kDelta);
            }

            @Override
            public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
                return Optional.of(new Rotation2d(3 * Math.PI / 4));
            }
        };
        VisionDataProvider24 vdp = new VisionDataProvider24(logger, layout, poseEstimator, f);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 2),
                new Rotation3d()));

        // test1 camera is tilted up 45 degrees
        final String cameraSerialNumber = "test1";
        final Blip24[] tags = new Blip24[] { tag4 };

        Experiments.instance.testOverride(Experiment.Triangulate, false);

        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
    }

    @Test
    void testCase7() throws IOException {

        // 1m in front of tag 4, 1m to the left, rotated to the right
        // looking up at a 30 degree angle
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void addVisionMeasurement(Pose2d p, double t, double[] sd1, double[] sd2) {
                assertEquals(0.96, p.getX(), kDelta);
                assertEquals(1.66, p.getY(), kDelta);
            }

            @Override
            public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
                return Optional.of(new Rotation2d(3 * Math.PI / 4));
            }
        };
        VisionDataProvider24 vdp = new VisionDataProvider24(logger, layout, poseEstimator, f);

        // 30 degrees, long side is sqrt2, so hypotenuse is sqrt2/sqrt3/2
        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.633),
                new Rotation3d()));

        // test3 camera is tilted up 30 degrees
        final String cameraSerialNumber = "test3";
        final Blip24[] tags = new Blip24[] { tag4 };

        Experiments.instance.testOverride(Experiment.Triangulate, false);

        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Alliance.Red);
    }
}