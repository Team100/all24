package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.ObjDoubleConsumer;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.sensors.HeadingWithHistory;
import org.team100.lib.sensors.MockHeading;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

class VisionDataProviderTest implements Timeless {
    private static final double kDelta = 0.01;

    @Test
    void testGetRobotPoseInFieldCoords2() {
        // trivial example: if camera offset happens to match the camera global pose
        // then of course the robot global pose is the origin.
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));
        Transform3d tagInCameraCoords = new Transform3d(new Translation3d(1, 0, 0), new Rotation3d());
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        Pose3d robotPoseInFieldCoords = PoseEstimationHelper.applyCameraOffset(
                cameraInFieldCoords,
                cameraInRobotCoords);

        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    void testGetRobotPoseInFieldCoords3() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));
        Translation3d tagTranslationInCameraCoords = new Translation3d(1, 0, 0);
        Rotation3d tagRotationInCameraCoords = new Rotation3d(0, 0, 0);
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        Pose3d robotPoseInFieldCoords = PoseEstimationHelper.applyCameraOffset(
                cameraInFieldCoords,
                cameraInRobotCoords);
        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    void testGetRobotPoseInFieldCoords4() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));

        Rotation3d cameraRotationInFieldCoords = new Rotation3d();

        Blip blip = new Blip(5,
                new double[][] { // pure tilt note we don't actually use this
                        { 1, 0, 0 },
                        { 0, 1, 0 },
                        { 0, 0, 1 } },
                new double[][] { // one meter range (Z forward)
                        { 0 },
                        { 0 },
                        { 1 } });
        Translation3d tagTranslationInCameraCoords = PoseEstimationHelper.blipToTranslation(blip);
        Rotation3d tagRotationInCameraCoords = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);

        Pose3d robotPoseInFieldCoords = PoseEstimationHelper.applyCameraOffset(
                cameraInFieldCoords,
                cameraInRobotCoords);
        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    void testEstimateRobotPose() throws IOException {
        // always at the origin
        DoubleFunction<Optional<Rotation2d>> robotRotation = (t) -> Optional.of(GeometryUtil.kRotationZero);
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .redLayout("2024-crescendo.json");
        // VisionDataProvider vdp = new VisionDataProvider(layout, null, robotPose);
        VisionDataProvider24 vdp = new VisionDataProvider24(layout, null, robotRotation);

        String key = "foo";
        // in red layout blip 7 is on the other side of the field

        // one meter range (Z forward)
        Blip24 blip = new Blip24(7, new Transform3d(new Translation3d(0, 0, 1), new Rotation3d()));

        // verify tag 5 location
        Pose3d tagPose = layout.getTagPose(7).get();
        assertEquals(16.5791, tagPose.getX(), kDelta);
        assertEquals(2.663, tagPose.getY(), kDelta);
        assertEquals(1.451, tagPose.getZ(), kDelta);
        assertEquals(0, tagPose.getRotation().getX(), kDelta);
        assertEquals(0, tagPose.getRotation().getY(), kDelta);
        assertEquals(0, tagPose.getRotation().getZ(), kDelta);

        Blip24[] blips = new Blip24[] {
                blip
        };
        final List<Pose2d> poseEstimate = new ArrayList<Pose2d>();
        final List<Double> timeEstimate = new ArrayList<Double>();
        vdp.estimateRobotPose(
                (p, t) -> {
                    poseEstimate.add(p);
                    timeEstimate.add(t);
                }, key, blips);
        // do it twice to convince vdp it's a good estimate
        vdp.estimateRobotPose(
                (p, t) -> {
                    poseEstimate.add(p);
                    timeEstimate.add(t);
                }, key, blips);
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
        DoubleFunction<Optional<Rotation2d>> robotRotation = (t) -> Optional.of(new Rotation2d(-Math.PI / 4));
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .redLayout("2024-crescendo.json");
        VisionDataProvider24 vdp = new VisionDataProvider24(layout, null, robotRotation);

        // camera sees the tag straight ahead in the center of the frame,
        // but rotated pi/4 to the left. this is ignored anyway.
        Blip24 blip = new Blip24(7, new Transform3d(
                new Translation3d(0, 0, Math.sqrt(2)),
                new Rotation3d(0, -Math.PI / 4, 0)));

        // verify tag 7 location
        Pose3d tagPose = layout.getTagPose(7).get();
        assertEquals(16.5791, tagPose.getX(), kDelta);
        assertEquals(2.663, tagPose.getY(), kDelta);
        assertEquals(1.451, tagPose.getZ(), kDelta);
        assertEquals(0, tagPose.getRotation().getX(), kDelta);
        assertEquals(0, tagPose.getRotation().getY(), kDelta);
        assertEquals(0, tagPose.getRotation().getZ(), kDelta);

        final List<Pose2d> poseEstimate = new ArrayList<Pose2d>();
        final List<Double> timeEstimate = new ArrayList<Double>();
        // default camera offset is no offset.
        String cameraSerialNumber = "foo";
        Blip24[] blips = new Blip24[] { blip };

        vdp.estimateRobotPose(
                (p, t) -> {
                    poseEstimate.add(p);
                    timeEstimate.add(t);
                }, cameraSerialNumber, blips);

        // two good estimates are required, so do another one.
        vdp.estimateRobotPose(
                (p, t) -> {
                    poseEstimate.add(p);
                    timeEstimate.add(t);
                }, cameraSerialNumber, blips);

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

    /**
     * The gyro delay is very small but the camera delay is long. I think combining
     * the measurements from these two sources is producing errors.
     * 
     * This is the same case as above, but with omega of about 10 rad/s (fast).
     * 
     * The way the pose estimator handles the delay is via replay, extrapolating
     * past poses at the time of the vision inputs. Since the network table input is
     * asynchronous, this extrapolation would have to happen in the pose supplier.
     */
    @Test
    void testDelay() throws IOException {
        MockHeading mock = new MockHeading();
        HeadingWithHistory hist = new HeadingWithHistory(mock);

        // rotation from the camera instant
        mock.rotation = new Rotation2d(-Math.PI / 4);

        // stick the current reading into the buffer.
        hist.periodic();

        // let 75ms pass.
        stepTime(0.075);

        // this rotation is from the current instant.
        // say the robot is turning at about 10 rad/s, say 10.472 for convenience.
        // then 75ms after the blip measurement below, the rotation is now zero.
        DoubleFunction<Optional<Rotation2d>> robotRotation = hist::sample;

        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .redLayout("2024-crescendo.json");
        VisionDataProvider24 vdp = new VisionDataProvider24(
                layout, null, robotRotation);

        // the robot *was* panned 45 right 75ms ago.
        // camera sees the tag straight ahead in the center of the frame,
        // but rotated pi/4 to the left. this is ignored anyway.
        // this blip is from 75ms ago.
        Blip24 blip = new Blip24(7, new Transform3d(
                new Translation3d(0, 0, Math.sqrt(2)),
                new Rotation3d(0, -Math.PI / 4, 0)));

        // verify tag 7 location
        Pose3d tagPose = layout.getTagPose(7).get();
        assertEquals(16.5791, tagPose.getX(), kDelta);
        assertEquals(2.663, tagPose.getY(), kDelta);
        assertEquals(1.451, tagPose.getZ(), kDelta);
        assertEquals(0, tagPose.getRotation().getX(), kDelta);
        assertEquals(0, tagPose.getRotation().getY(), kDelta);
        assertEquals(0, tagPose.getRotation().getZ(), kDelta);

        final List<Pose2d> poseEstimate = new ArrayList<Pose2d>();
        final List<Double> timeEstimate = new ArrayList<Double>();
        // default camera offset is no offset.
        String cameraSerialNumber = "foo";
        Blip24[] blips = new Blip24[] { blip };

        vdp.estimateRobotPose(
                (p, t) -> {
                    poseEstimate.add(p);
                    timeEstimate.add(t);
                }, cameraSerialNumber, blips);

        // two good estimates are required, so do another one.
        vdp.estimateRobotPose(
                (p, t) -> {
                    poseEstimate.add(p);
                    timeEstimate.add(t);
                }, cameraSerialNumber, blips);

        assertEquals(1, poseEstimate.size());
        assertEquals(1, timeEstimate.size());

        Pose2d result = poseEstimate.get(0);
        // robot is is one meter away from the target in x
        // without correction: 15.075
        assertEquals(15.5791, result.getX(), kDelta);
        // robot is one meter to the left (i.e. in y)
        // without correction: 2.663
        assertEquals(3.663, result.getY(), kDelta);
        // facing diagonal, this is just what we provided.
        // without correction: 0
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
        System.out.println("test case 1");
        // the case from 2/14
        // robot 45 degrees to the right (negative), so 135 degrees
        // x = 2.2m, y = - 1.3 m from the center speaker tag
        // camera B
        // camera to tag 4: z=2.4, x=0, y=0 (approx)
        // camera to tag 3: z=2.8, x=0.1, y=0.1 (approx)
        // tag 4 in red is at about (0, 2.5)
        // tag 3 in red is at about (0, 3)

        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .redLayout("2024-crescendo.json");
        DoubleFunction<Optional<Rotation2d>> rotationSupplier = (t) -> Optional.of(new Rotation2d(3 * Math.PI / 4));
        VisionDataProvider24 vdp = new VisionDataProvider24(layout, null, rotationSupplier);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 2.4),
                new Rotation3d()));
        Blip24 tag3 = new Blip24(3, new Transform3d(
                new Translation3d(0.1, 0.1, 2.8),
                new Rotation3d()));
        Blip24[] tags = new Blip24[] { tag3, tag4 };

        ObjDoubleConsumer<Pose2d> estimateConsumer = (coord, time) -> {
            System.out.println(coord);
        };

        Experiments.instance.testOverride(Experiment.Triangulate, true);
        vdp.estimateRobotPose(estimateConsumer, "1000000013c9c96c", tags);
        vdp.estimateRobotPose(estimateConsumer, "1000000013c9c96c", tags);
    }

    @Test
    void testCase2() throws IOException {
        // 1m in front of tag 4
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .redLayout("2024-crescendo.json");
        Pose3d tag4pose = layout.getTagPose(4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        DoubleFunction<Optional<Rotation2d>> rotationSupplier = (t) -> Optional.of(new Rotation2d(Math.PI));
        VisionDataProvider24 vdp = new VisionDataProvider24(layout, null, rotationSupplier);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));
        Blip24[] tags = new Blip24[] { tag4 };

        ObjDoubleConsumer<Pose2d> estimateConsumer = (coord, time) -> {
            assertEquals(0.96, coord.getX(), kDelta);
            assertEquals(2.66, coord.getY(), kDelta);
        };

        Experiments.instance.testOverride(Experiment.Triangulate, false);
        // test camera has zero offset
        vdp.estimateRobotPose(estimateConsumer, "test", tags);
        vdp.estimateRobotPose(estimateConsumer, "test", tags);
    }

        @Test
    void testCase2WithOffset() throws IOException {
        // 1m in front of tag 4
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .redLayout("2024-crescendo.json");
        Pose3d tag4pose = layout.getTagPose(4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        DoubleFunction<Optional<Rotation2d>> rotationSupplier = (t) -> Optional.of(new Rotation2d(Math.PI));
        VisionDataProvider24 vdp = new VisionDataProvider24(layout, null, rotationSupplier);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));
        Blip24[] tags = new Blip24[] { tag4 };

        ObjDoubleConsumer<Pose2d> estimateConsumer = (coord, time) -> {
            assertEquals(1.96, coord.getX(), kDelta);
            assertEquals(2.66, coord.getY(), kDelta);
        };

        Experiments.instance.testOverride(Experiment.Triangulate, false);
        // test2 camera is 1m in front, so robot is 1m further away.
        vdp.estimateRobotPose(estimateConsumer, "test2", tags);
        vdp.estimateRobotPose(estimateConsumer, "test2", tags);
    }

    @Test
    void testCase2WithTriangulation() throws IOException {
        // 1m in front of tag 4
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .redLayout("2024-crescendo.json");
        Pose3d tag4pose = layout.getTagPose(4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        DoubleFunction<Optional<Rotation2d>> rotationSupplier = (t) -> Optional.of(new Rotation2d(Math.PI));
        VisionDataProvider24 vdp = new VisionDataProvider24(layout, null, rotationSupplier);

        Blip24 tag3 = new Blip24(3, new Transform3d(
                new Translation3d(0.561, 0, 1),
                new Rotation3d()));
        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));
        Blip24[] tags = new Blip24[] { tag3, tag4 };

        ObjDoubleConsumer<Pose2d> estimateConsumer = (coord, time) -> {
            System.out.println(coord);
            assertEquals(0.96, coord.getX(), kDelta);
            assertEquals(2.66, coord.getY(), kDelta);
        };

        Experiments.instance.testOverride(Experiment.Triangulate, true);
        // test camera has zero offset
        vdp.estimateRobotPose(estimateConsumer, "test", tags);
        vdp.estimateRobotPose(estimateConsumer, "test", tags);
    }

    @Test
    void testCase2tilt() throws IOException {
        // 1m in front of tag 4, tilted up 45
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .redLayout("2024-crescendo.json");
        Pose3d tag4pose = layout.getTagPose(4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        DoubleFunction<Optional<Rotation2d>> rotationSupplier = (t) -> Optional.of(new Rotation2d(Math.PI));
        VisionDataProvider24 vdp = new VisionDataProvider24(layout, null, rotationSupplier);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));
        Blip24[] tags = new Blip24[] { tag4 };

        ObjDoubleConsumer<Pose2d> estimateConsumer = (coord, time) -> {
            assertEquals(0.96, coord.getX(), kDelta);
            assertEquals(2.66, coord.getY(), kDelta);
        };

        Experiments.instance.testOverride(Experiment.Triangulate, false);
        // test camera has zero offset
        vdp.estimateRobotPose(estimateConsumer, "test1", tags);
        vdp.estimateRobotPose(estimateConsumer, "test1", tags);
    }

    @Test
    void testCase3() throws IOException {
        // 1m in front of tag 4, 1m to the right
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .redLayout("2024-crescendo.json");
        Pose3d tag4pose = layout.getTagPose(4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        DoubleFunction<Optional<Rotation2d>> rotationSupplier = (t) -> Optional.of(new Rotation2d(Math.PI));
        VisionDataProvider24 vdp = new VisionDataProvider24(layout, null, rotationSupplier);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(-1, 0, 1),
                new Rotation3d()));
        Blip24[] tags = new Blip24[] { tag4 };

        ObjDoubleConsumer<Pose2d> estimateConsumer = (coord, time) -> {
            assertEquals(0.96, coord.getX(), kDelta);
            assertEquals(3.66, coord.getY(), kDelta);
        };

        Experiments.instance.testOverride(Experiment.Triangulate, false);
        // test camera has zero offset
        vdp.estimateRobotPose(estimateConsumer, "test", tags);
        vdp.estimateRobotPose(estimateConsumer, "test", tags);
    }

    @Test
    void testCase4() throws IOException {
        // 1m in front of tag 4, 1m to the right, rotated to the left
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .redLayout("2024-crescendo.json");
        Pose3d tag4pose = layout.getTagPose(4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        DoubleFunction<Optional<Rotation2d>> rotationSupplier = (t) -> Optional.of(new Rotation2d(-3 * Math.PI / 4));
        VisionDataProvider24 vdp = new VisionDataProvider24(layout, null, rotationSupplier);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));
        Blip24[] tags = new Blip24[] { tag4 };

        ObjDoubleConsumer<Pose2d> estimateConsumer = (coord, time) -> {
            System.out.println(coord);
            assertEquals(0.96, coord.getX(), kDelta);
            assertEquals(3.66, coord.getY(), kDelta);
        };

        Experiments.instance.testOverride(Experiment.Triangulate, false);
        // test camera has zero offset
        vdp.estimateRobotPose(estimateConsumer, "test", tags);
        vdp.estimateRobotPose(estimateConsumer, "test", tags);
    }

    @Test
    void testCase5() throws IOException {
        // 1m in front of tag 4, 1m to the left, rotated to the right
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .redLayout("2024-crescendo.json");
        Pose3d tag4pose = layout.getTagPose(4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        DoubleFunction<Optional<Rotation2d>> rotationSupplier = (t) -> Optional.of(new Rotation2d(3 * Math.PI / 4));
        VisionDataProvider24 vdp = new VisionDataProvider24(layout, null, rotationSupplier);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));
        Blip24[] tags = new Blip24[] { tag4 };

        ObjDoubleConsumer<Pose2d> estimateConsumer = (coord, time) -> {
            System.out.println(coord);
            assertEquals(0.96, coord.getX(), kDelta);
            assertEquals(1.66, coord.getY(), kDelta);
        };

        Experiments.instance.testOverride(Experiment.Triangulate, false);
        // test camera has zero offset
        vdp.estimateRobotPose(estimateConsumer, "test", tags);
        vdp.estimateRobotPose(estimateConsumer, "test", tags);
    }

    @Test
    void testCase6() throws IOException {
        // 1m in front of tag 4, 1m to the left, rotated to the right
        // looking up at a 45 degree angle
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .redLayout("2024-crescendo.json");
        Pose3d tag4pose = layout.getTagPose(4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        DoubleFunction<Optional<Rotation2d>> rotationSupplier = (t) -> Optional.of(new Rotation2d(3 * Math.PI / 4));
        VisionDataProvider24 vdp = new VisionDataProvider24(layout, null, rotationSupplier);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 2),
                new Rotation3d()));
        Blip24[] tags = new Blip24[] { tag4 };

        ObjDoubleConsumer<Pose2d> estimateConsumer = (coord, time) -> {
            System.out.println(coord);
            assertEquals(0.96, coord.getX(), kDelta);
            assertEquals(1.66, coord.getY(), kDelta);
        };

        Experiments.instance.testOverride(Experiment.Triangulate, false);
        // test1 camera is tilted up 45 degrees
        vdp.estimateRobotPose(estimateConsumer, "test1", tags);
        vdp.estimateRobotPose(estimateConsumer, "test1", tags);
    }

    @Test
    void testCase7() throws IOException {
        // 1m in front of tag 4, 1m to the left, rotated to the right
        // looking up at a 30 degree angle
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .redLayout("2024-crescendo.json");
        Pose3d tag4pose = layout.getTagPose(4).get();
        assertEquals(-0.038, tag4pose.getX(), kDelta);
        assertEquals(2.663, tag4pose.getY(), kDelta);
        assertEquals(1.451, tag4pose.getZ(), kDelta);

        DoubleFunction<Optional<Rotation2d>> rotationSupplier = (t) -> Optional.of(new Rotation2d(3 * Math.PI / 4));
        VisionDataProvider24 vdp = new VisionDataProvider24(layout, null, rotationSupplier);

        // 30 degrees, long side is sqrt2, so hypotenuse is sqrt2/sqrt3/2
        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.633),
                new Rotation3d()));
        Blip24[] tags = new Blip24[] { tag4 };

        ObjDoubleConsumer<Pose2d> estimateConsumer = (coord, time) -> {
            System.out.println(coord);
            assertEquals(0.96, coord.getX(), kDelta);
            assertEquals(1.66, coord.getY(), kDelta);
        };

        Experiments.instance.testOverride(Experiment.Triangulate, false);
        // test3 camera is tilted up 30 degrees
        vdp.estimateRobotPose(estimateConsumer, "test3", tags);
        vdp.estimateRobotPose(estimateConsumer, "test3", tags);
    }
}
