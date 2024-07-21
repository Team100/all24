package org.team100.lib.config;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.localization.TargetLocalizer;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

class SimulatedCameraTest {
    private static final double kDelta = 0.001;

    @Test
    void testNotePose() {
        // camera is 1m up, down 45
        Transform3d cameraInRobotCoordinates = new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d(0, Math.toRadians(45), 0));
        SimulatedCamera simCamera = new SimulatedCamera(
                cameraInRobotCoordinates,
                Math.toRadians(40),
                Math.toRadians(31.5));

        Pose2d robotPose = new Pose2d();

        // goal is straight ahead, same as height, so should be on-bore
        Translation2d[] notes = { new Translation2d(1, 0) };
        List<Rotation3d> sights = simCamera.getRotations(robotPose, notes);

        assertEquals(1, sights.size());
        // on-bore
        assertEquals(0, sights.get(0).getX(), kDelta);
        assertEquals(0, sights.get(0).getY(), kDelta);
        assertEquals(0, sights.get(0).getZ(), kDelta);

        List<Translation2d> targets = TargetLocalizer.cameraRotsToFieldRelativeArray(
                robotPose,
                cameraInRobotCoordinates,
                sights.toArray(new Rotation3d[0]));
        assertEquals(1, targets.size());
        // we should get back the goal
        assertEquals(1, targets.get(0).getX(), kDelta);
        assertEquals(0, targets.get(0).getY(), kDelta);
    }

    @Test
    void testNotePose2() {
        // camera is 1m up, level
        Transform3d cameraInRobotCoordinates = new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d());
        SimulatedCamera simCamera = new SimulatedCamera(
                cameraInRobotCoordinates,
                Math.toRadians(40),
                Math.toRadians(31.5));

        Pose2d robotPose = new Pose2d();

        // goal is straight ahead, 2x further than height, so ~26 degrees down
        Translation2d[] notes = { new Translation2d(2, 0) };

        List<Rotation3d> sights = simCamera.getRotations(robotPose, notes);

        assertEquals(1, sights.size());
        // on-bore
        assertEquals(0, sights.get(0).getX(), kDelta);
        assertEquals(0.463, sights.get(0).getY(), kDelta);
        assertEquals(0, sights.get(0).getZ(), kDelta);

        List<Translation2d> targets = TargetLocalizer.cameraRotsToFieldRelativeArray(
                robotPose,
                cameraInRobotCoordinates,
                sights.toArray(new Rotation3d[0]));
        assertEquals(1, targets.size());
        // we should get back the goal
        assertEquals(2, targets.get(0).getX(), kDelta);
        assertEquals(0, targets.get(0).getY(), kDelta);
    }

    @Test
    void testNotePose3() {
        // camera is 1m up, level
        Transform3d cameraInRobotCoordinates = new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d());
        SimulatedCamera simCamera = new SimulatedCamera(
                cameraInRobotCoordinates,
                Math.toRadians(40),
                Math.toRadians(31.5));

        Pose2d robotPose = new Pose2d();

        // note is to the left, 2x further than height, so ~26 degrees
        Translation2d[] notes = { new Translation2d(2, 1) };
        List<Rotation3d> sights = simCamera.getRotations(robotPose, notes);

        assertEquals(1, sights.size());
        // roll?
        assertEquals(0.101, sights.get(0).getX(), kDelta);
        // less than the expected
        assertEquals(0.420, sights.get(0).getY(), kDelta);
        // exactly the expected
        assertEquals(0.463, sights.get(0).getZ(), kDelta);

        List<Translation2d> targets = TargetLocalizer.cameraRotsToFieldRelativeArray(
                robotPose,
                cameraInRobotCoordinates,
                sights.toArray(new Rotation3d[0]));
        assertEquals(1, targets.size());
        // we should get back the goal
        assertEquals(2, targets.get(0).getX(), kDelta);
        assertEquals(1, targets.get(0).getY(), kDelta);
    }

    @Test
    void testNotePose4() {
        // camera is 1m up, level
        Transform3d cameraInRobotCoordinates = new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d());
        SimulatedCamera simCamera = new SimulatedCamera(
                cameraInRobotCoordinates,
                Math.toRadians(40),
                Math.toRadians(31.5));

        // note is to the left, 2x further than height, so ~26 degrees
        Translation2d[] notes = { new Translation2d(2, 1) };

        // this is what the camera should see.
        Rotation3d expectedSight = new Rotation3d(VecBuilder.fill(2, 0, 0), VecBuilder.fill(2, 1, -1));
        // irrelevant roll
        assertEquals(0.101, expectedSight.getX(), kDelta);
        // a bit less than the expected pitch, since the yaw pulls it in
        assertEquals(0.420, expectedSight.getY(), kDelta);
        // expected yaw
        assertEquals(0.463, expectedSight.getZ(), kDelta);

        Pose2d robotPose = new Pose2d();

        List<Rotation3d> sights = simCamera.getRotations(robotPose, notes);

        assertEquals(1, sights.size());
        // roll doesn't matter
        assertEquals(0.101, sights.get(0).getX(), kDelta);
        assertEquals(0.420, sights.get(0).getY(), kDelta);
        assertEquals(0.463, sights.get(0).getZ(), kDelta);

        List<Translation2d> targets = TargetLocalizer.cameraRotsToFieldRelativeArray(
                robotPose,
                cameraInRobotCoordinates,
                sights.toArray(new Rotation3d[0]));
        assertEquals(1, targets.size());
        // we should get back the goal
        assertEquals(2, targets.get(0).getX(), kDelta);
        assertEquals(1, targets.get(0).getY(), kDelta);
    }

    @Test
    void testNotePose5() {
        // camera is 1m up, 90 left
        Transform3d cameraInRobotCoordinates = new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d(0, 0, Math.PI / 2));
        SimulatedCamera simCamera = new SimulatedCamera(
                cameraInRobotCoordinates,
                Math.toRadians(40),
                Math.toRadians(31.5));

        Pose2d robotPose = new Pose2d();

        Translation2d[] notes = { new Translation2d(0, 2) };
        List<Rotation3d> sights = simCamera.getRotations(robotPose, notes);

        assertEquals(1, sights.size());
        // camera should see target straight ahead
        assertEquals(0, sights.get(0).getX(), kDelta);
        assertEquals(0.463, sights.get(0).getY(), kDelta);
        assertEquals(0, sights.get(0).getZ(), kDelta);

        List<Translation2d> targets = TargetLocalizer.cameraRotsToFieldRelativeArray(
                robotPose,
                cameraInRobotCoordinates,
                sights.toArray(new Rotation3d[0]));
        assertEquals(1, targets.size());
        // we should get back the goal
        assertEquals(0, targets.get(0).getX(), kDelta);
        assertEquals(2, targets.get(0).getY(), kDelta);
    }

    @Test
    void testNotePose6() {
        // from getRotation.
        {
            Pose2d robotPose = new Pose2d();
            Pose2d notePose = new Pose2d(new Translation2d(2, 0), new Rotation2d());
            Translation2d relative = notePose.relativeTo(robotPose).getTranslation();
            assertEquals(2, relative.getX(), kDelta);
            assertEquals(0, relative.getY(), kDelta);
        }
        {
            // robot facing left
            Pose2d robotPose = new Pose2d(new Translation2d(), new Rotation2d(Math.PI / 2));
            Pose2d notePose = new Pose2d(new Translation2d(2, 0), new Rotation2d());
            Translation2d relative = notePose.relativeTo(robotPose).getTranslation();
            assertEquals(0, relative.getX(), kDelta);
            assertEquals(-2, relative.getY(), kDelta);
        }

        {
            // robot at origin
            Pose2d robotPose = new Pose2d();
            // note 2m ahead
            Pose2d notePose = new Pose2d(new Translation2d(2, 0), new Rotation2d());
            Translation2d relative = notePose.relativeTo(robotPose).getTranslation();
            assertEquals(2, relative.getX(), kDelta);
            assertEquals(0, relative.getY(), kDelta);
            Transform3d noteInRobotCoords = new Transform3d(
                    new Translation3d(relative.getX(), relative.getY(), 0),
                    new Rotation3d());
            // camera 1m up
            Transform3d cameraInRobotCoordinates = new Transform3d(new Translation3d(0, 0, 1), new Rotation3d());
            Transform3d robotInCameraCoordinates = cameraInRobotCoordinates.inverse();
            Transform3d noteInCameraCoordinates = noteInRobotCoords.plus(robotInCameraCoordinates);
            // 2m ahead
            assertEquals(2, noteInCameraCoordinates.getX(), kDelta);
            assertEquals(0, noteInCameraCoordinates.getY(), kDelta);
            // 1m down from camera
            assertEquals(-1, noteInCameraCoordinates.getZ(), kDelta);
            // no rotation
            assertEquals(0, noteInCameraCoordinates.getRotation().getX(), kDelta);
            assertEquals(0, noteInCameraCoordinates.getRotation().getY(), kDelta);
            assertEquals(0, noteInCameraCoordinates.getRotation().getZ(), kDelta);
        }
    }

    @Test
    void testNotePose7() {
        {
            // robot at origin
            Pose2d robotPose = new Pose2d();

            // camera 1m up, level
            Transform3d cameraInRobotCoordinates = new Transform3d(
                    new Translation3d(0, 0, 1), new Rotation3d());

            Translation2d note = new Translation2d(1, 0);

            SimulatedCamera cam = new SimulatedCamera(cameraInRobotCoordinates, Math.PI / 2, Math.PI / 2);
            Transform3d noteInCameraCoordinates = cam.getNoteInCameraCoordinates(robotPose, note);

            // 1m ahead
            assertEquals(1, noteInCameraCoordinates.getX(), kDelta);
            assertEquals(0, noteInCameraCoordinates.getY(), kDelta);
            // 1m down from camera
            assertEquals(-1, noteInCameraCoordinates.getZ(), kDelta);
            // no rotation
            assertEquals(0, noteInCameraCoordinates.getRotation().getX(), kDelta);
            assertEquals(0, noteInCameraCoordinates.getRotation().getY(), kDelta);
            assertEquals(0, noteInCameraCoordinates.getRotation().getZ(), kDelta);

            Optional<Rotation3d> rotInCamera = cam.getRotInCamera(robotPose, note);

            // roll is irrelevant anyway
            assertEquals(0, rotInCamera.get().getX(), kDelta);
            // this should be atan(1) = 0.785
            assertEquals(0.785, rotInCamera.get().getY(), kDelta);
            // dead ahead
            assertEquals(0, rotInCamera.get().getZ(), kDelta);

            // do we get back the original pose?

            Optional<Translation2d> target = TargetLocalizer.cameraRotToFieldRelative(
                    robotPose,
                    cameraInRobotCoordinates,
                    rotInCamera.get());
            // we should get back the goal
            assertEquals(1, target.get().getX(), kDelta);
            assertEquals(0, target.get().getY(), kDelta);
        }
    }

    @Test
    void testNotePose7a() {
        {
            // this is the same case as above, but further away
            // robot at origin
            Pose2d robotPose = new Pose2d();

            // camera 1m up, level
            Transform3d cameraInRobotCoordinates = new Transform3d(
                    new Translation3d(0, 0, 1), new Rotation3d());

            // further away
            Translation2d note = new Translation2d(2, 0);

            SimulatedCamera cam = new SimulatedCamera(cameraInRobotCoordinates, Math.PI / 2, Math.PI / 2);
            Transform3d noteInCameraCoordinates = cam.getNoteInCameraCoordinates(robotPose, note);

            // 2m ahead
            assertEquals(2, noteInCameraCoordinates.getX(), kDelta);
            assertEquals(0, noteInCameraCoordinates.getY(), kDelta);
            // 1m down from camera
            assertEquals(-1, noteInCameraCoordinates.getZ(), kDelta);
            // no rotation
            assertEquals(0, noteInCameraCoordinates.getRotation().getX(), kDelta);
            assertEquals(0, noteInCameraCoordinates.getRotation().getY(), kDelta);
            assertEquals(0, noteInCameraCoordinates.getRotation().getZ(), kDelta);

            Optional<Rotation3d> rotInCamera = cam.getRotInCamera(robotPose, note);

            // roll is irrelevant anyway
            assertEquals(0, rotInCamera.get().getX(), kDelta);
            // this should be atan(0.5) = 0.463
            assertEquals(0.463, rotInCamera.get().getY(), kDelta);
            // dead ahead
            assertEquals(0, rotInCamera.get().getZ(), kDelta);

            // do we get back the original pose?

            Optional<Translation2d> target = TargetLocalizer.cameraRotToFieldRelative(
                    robotPose,
                    cameraInRobotCoordinates,
                    rotInCamera.get());
            // we should get back the goal
            assertEquals(2, target.get().getX(), kDelta);
            assertEquals(0, target.get().getY(), kDelta);
        }
    }

    @Test
    void testNotePose8() {
        {
            // robot at origin
            Pose2d robotPose = new Pose2d();

            // camera 1m up, looking left
            Transform3d cameraInRobotCoordinates = new Transform3d(
                    new Translation3d(0, 0, 1), new Rotation3d(0, 0, Math.PI / 2));

            Translation2d note = new Translation2d(0, 2);

            SimulatedCamera cam = new SimulatedCamera(cameraInRobotCoordinates, Math.PI / 2, Math.PI / 2);
            Transform3d noteInCameraCoordinates = cam.getNoteInCameraCoordinates(robotPose, note);

            // 2m ahead
            assertEquals(2, noteInCameraCoordinates.getX(), kDelta);
            assertEquals(0, noteInCameraCoordinates.getY(), kDelta);
            // 1m down from camera
            assertEquals(-1, noteInCameraCoordinates.getZ(), kDelta);
            assertEquals(0, noteInCameraCoordinates.getRotation().getX(), kDelta);
            assertEquals(0, noteInCameraCoordinates.getRotation().getY(), kDelta);
            // -pi/2 yaw, doesn't matter
            assertEquals(-1.571, noteInCameraCoordinates.getRotation().getZ(), kDelta);
        }
    }

    @Test
    void testNotePose9() {
        {
            // robot at origin
            Pose2d robotPose = new Pose2d();

            // camera 1m up, looking left
            Transform3d cameraInRobotCoordinates = new Transform3d(
                    new Translation3d(0, 0, 1), new Rotation3d(0, Math.PI / 4, Math.PI / 2));

            Translation2d note = new Translation2d(0, 2);
            SimulatedCamera cam = new SimulatedCamera(cameraInRobotCoordinates, Math.PI / 2, Math.PI / 2);
            Transform3d noteInCameraCoordinates = cam.getNoteInCameraCoordinates(robotPose, note);

            // a bit more than 2m ahead
            assertEquals(2.121, noteInCameraCoordinates.getX(), kDelta);
            // dead ahead
            assertEquals(0, noteInCameraCoordinates.getY(), kDelta);
            // above the bore
            assertEquals(0.707, noteInCameraCoordinates.getZ(), kDelta);
            // irrelevant
            assertEquals(0.785, noteInCameraCoordinates.getRotation().getX(), kDelta);
            assertEquals(0, noteInCameraCoordinates.getRotation().getY(), kDelta);
            // -pi/2 yaw, doesn't matter
            assertEquals(-1.571, noteInCameraCoordinates.getRotation().getZ(), kDelta);
        }
    }

    @Test
    void testNotePose10() {
        {
            // robot at origin
            Pose2d robotPose = new Pose2d();
            // camera 1m up, looking left
            Transform3d cameraInRobotCoordinates = new Transform3d(
                    new Translation3d(0, 0, 1), new Rotation3d(0, Math.PI / 4, Math.PI / 2));

            Translation2d note = new Translation2d(0, 2);

            SimulatedCamera cam = new SimulatedCamera(cameraInRobotCoordinates, Math.PI / 2, Math.PI / 2);

            Optional<Rotation3d> rotInCamera = cam.getRotInCamera(robotPose, note);

            // roll is irrelevant anyway
            assertEquals(0, rotInCamera.get().getX(), kDelta);
            // a little pitch up
            assertEquals(-0.321, rotInCamera.get().getY(), kDelta);
            // dead ahead
            assertEquals(0, rotInCamera.get().getZ(), kDelta);

            // do we get back the original pose?

            Optional<Translation2d> target = TargetLocalizer.cameraRotToFieldRelative(
                    robotPose,
                    cameraInRobotCoordinates,
                    rotInCamera.get());
            // we should get back the goal
            assertEquals(0, target.get().getX(), kDelta);
            assertEquals(2, target.get().getY(), kDelta);
        }
    }

    @Test
    void testNotePose11() {
        {
            // robot at origin
            Pose2d robotPose = new Pose2d();
            // camera 1m up, looking left
            Transform3d cameraInRobotCoordinates = new Transform3d(
                    new Translation3d(0, 0, 1), new Rotation3d(0, Math.PI / 4, Math.PI / 2));

            Translation2d note = new Translation2d(0, 1);

            SimulatedCamera cam = new SimulatedCamera(cameraInRobotCoordinates, Math.PI / 2, Math.PI / 2);
            Optional<Rotation3d> rotInCamera = cam.getRotInCamera(robotPose, note);

            // roll is irrelevant anyway
            assertEquals(0, rotInCamera.get().getX(), kDelta);
            // on bore
            assertEquals(0, rotInCamera.get().getY(), kDelta);
            // dead ahead
            assertEquals(0, rotInCamera.get().getZ(), kDelta);

            // do we get back the original pose?

            Optional<Translation2d> target = TargetLocalizer.cameraRotToFieldRelative(
                    robotPose,
                    cameraInRobotCoordinates,
                    rotInCamera.get());
            // we should get back the goal
            assertEquals(0, target.get().getX(), kDelta);
            assertEquals(1, target.get().getY(), kDelta);
        }
    }
}
